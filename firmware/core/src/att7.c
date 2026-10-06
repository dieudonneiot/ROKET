#include "fc/att7.h"

#include <math.h>
#include <string.h>

_Static_assert(sizeof(float) == 4u, "Attachment-7 floats are IEEE-754 binary32");

#define ATT7_FIELD_COUNT 8u

/* Beyond this magnitude a float has no fractional digits left to round. */
#define ATT7_ROUND_LIMIT 1.0e7f

void att7_config_default(att7_config_t *config)
{
    if (config == NULL) {
        return;
    }
    config->tx_checksum = ATT7_CHECKSUM_WITH_HEADER;
    config->float_order = ATT7_FLOAT_BIG_ENDIAN;
    config->round_2dp = 1u;
    config->strict_rx_checksum = 0u;
    config->status_low_first = 1u;
}

uint8_t att7_checksum(const uint8_t *frame, size_t checksum_index, uint8_t mode)
{
    size_t i = (mode == ATT7_CHECKSUM_WITHOUT_HEADER) ? 1u : 0u;
    uint32_t sum = 0u;

    if (frame == NULL) {
        return 0u;
    }
    for (; i < checksum_index; i++) {
        sum += frame[i];
    }
    return (uint8_t)(sum & 0xFFu);
}

float att7_round_2dp(float value)
{
    float rounded;

    if (!isfinite(value) || fabsf(value) >= ATT7_ROUND_LIMIT) {
        return value;
    }
    rounded = roundf(value * 100.0f) / 100.0f;
    return (rounded == 0.0f) ? 0.0f : rounded;
}

static void put_f32(uint8_t *dst, float value, uint8_t order)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof bits);
    if (order == ATT7_FLOAT_LITTLE_ENDIAN) {
        dst[0] = (uint8_t)(bits & 0xFFu);
        dst[1] = (uint8_t)((bits >> 8) & 0xFFu);
        dst[2] = (uint8_t)((bits >> 16) & 0xFFu);
        dst[3] = (uint8_t)((bits >> 24) & 0xFFu);
    } else {
        dst[0] = (uint8_t)((bits >> 24) & 0xFFu);
        dst[1] = (uint8_t)((bits >> 16) & 0xFFu);
        dst[2] = (uint8_t)((bits >> 8) & 0xFFu);
        dst[3] = (uint8_t)(bits & 0xFFu);
    }
}

static float get_f32(const uint8_t *src, uint8_t order)
{
    uint32_t bits;
    float value;

    if (order == ATT7_FLOAT_LITTLE_ENDIAN) {
        bits = (uint32_t)src[0] | ((uint32_t)src[1] << 8) | ((uint32_t)src[2] << 16) |
               ((uint32_t)src[3] << 24);
    } else {
        bits = ((uint32_t)src[0] << 24) | ((uint32_t)src[1] << 16) | ((uint32_t)src[2] << 8) |
               (uint32_t)src[3];
    }
    memcpy(&value, &bits, sizeof value);
    return value;
}

static void sample_to_fields(const att7_sample_t *sample, float fields[ATT7_FIELD_COUNT])
{
    fields[0] = sample->altitude_m;
    fields[1] = sample->pressure_mbar;
    fields[2] = sample->accel_x_mps2;
    fields[3] = sample->accel_y_mps2;
    fields[4] = sample->accel_z_mps2;
    fields[5] = sample->angle_x_deg;
    fields[6] = sample->angle_y_deg;
    fields[7] = sample->angle_z_deg;
}

static void fields_to_sample(const float fields[ATT7_FIELD_COUNT], att7_sample_t *sample)
{
    sample->altitude_m = fields[0];
    sample->pressure_mbar = fields[1];
    sample->accel_x_mps2 = fields[2];
    sample->accel_y_mps2 = fields[3];
    sample->accel_z_mps2 = fields[4];
    sample->angle_x_deg = fields[5];
    sample->angle_y_deg = fields[6];
    sample->angle_z_deg = fields[7];
}

size_t att7_encode_command(uint8_t command, uint8_t checksum_style, uint8_t out[ATT7_COMMAND_LEN])
{
    if (out == NULL) {
        return 0u;
    }
    out[0] = ATT7_HEADER_COMMAND;
    out[1] = command;
    if (checksum_style == ATT7_COMMAND_CHECKSUM_TABLE1) {
        out[2] = (uint8_t)((command + ATT7_TABLE1_CHECKSUM_OFFSET) & 0xFFu);
    } else {
        out[2] = (uint8_t)((ATT7_HEADER_COMMAND + command) & 0xFFu);
    }
    out[3] = ATT7_FOOTER_1;
    out[4] = ATT7_FOOTER_2;
    return ATT7_COMMAND_LEN;
}

size_t att7_encode_data(const att7_config_t *config, const att7_sample_t *sample,
                        uint8_t out[ATT7_DATA_LEN])
{
    float fields[ATT7_FIELD_COUNT];
    size_t i;

    if (config == NULL || sample == NULL || out == NULL) {
        return 0u;
    }
    sample_to_fields(sample, fields);
    out[0] = ATT7_HEADER_DATA;
    for (i = 0u; i < ATT7_FIELD_COUNT; i++) {
        float value = (config->round_2dp != 0u) ? att7_round_2dp(fields[i]) : fields[i];
        put_f32(&out[1u + 4u * i], value, config->float_order);
    }
    out[ATT7_DATA_CHECKSUM_INDEX] =
        att7_checksum(out, ATT7_DATA_CHECKSUM_INDEX, config->tx_checksum);
    out[ATT7_DATA_LEN - 2u] = ATT7_FOOTER_1;
    out[ATT7_DATA_LEN - 1u] = ATT7_FOOTER_2;
    return ATT7_DATA_LEN;
}

size_t att7_encode_status(const att7_config_t *config, uint16_t status,
                          uint8_t out[ATT7_STATUS_LEN])
{
    uint8_t low = (uint8_t)(status & 0xFFu);
    uint8_t high = (uint8_t)((status >> 8) & 0xFFu);

    if (config == NULL || out == NULL) {
        return 0u;
    }
    out[0] = ATT7_HEADER_COMMAND;
    out[1] = (config->status_low_first != 0u) ? low : high;
    out[2] = (config->status_low_first != 0u) ? high : low;
    out[ATT7_STATUS_CHECKSUM_INDEX] =
        att7_checksum(out, ATT7_STATUS_CHECKSUM_INDEX, config->tx_checksum);
    out[4] = ATT7_FOOTER_1;
    out[5] = ATT7_FOOTER_2;
    return ATT7_STATUS_LEN;
}

/* ---- Parser --------------------------------------------------------------- */

static uint8_t expected_length(uint8_t header)
{
    if (header == ATT7_HEADER_COMMAND) {
        return (uint8_t)ATT7_COMMAND_LEN;
    }
    if (header == ATT7_HEADER_DATA) {
        return (uint8_t)ATT7_DATA_LEN;
    }
    return 0u;
}

static uint8_t is_known_command(uint8_t command)
{
    return (uint8_t)(command == ATT7_CMD_START_SMT || command == ATT7_CMD_START_SFT ||
                     command == ATT7_CMD_STOP);
}

/* Drops the first `count` buffered bytes, then re-aligns on the next header byte. */
static void discard(att7_parser_t *parser, uint8_t count)
{
    uint8_t start = count;

    while (start < parser->len && expected_length(parser->buf[start]) == 0u) {
        start++;
    }
    if (start >= parser->len) {
        parser->len = 0u;
        parser->expected = 0u;
        return;
    }
    memmove(parser->buf, &parser->buf[start], (size_t)(parser->len - start));
    parser->len = (uint8_t)(parser->len - start);
    parser->expected = expected_length(parser->buf[0]);
}

/* Returns 1 and fills *event if buf[0 .. expected) is an acceptable frame. */
static uint8_t decode_frame(att7_parser_t *parser, const att7_config_t *config, att7_event_t *event)
{
    const uint8_t *frame = parser->buf;
    uint8_t length = parser->expected;
    uint8_t checksum_ok;

    if (frame[length - 2u] != ATT7_FOOTER_1 || frame[length - 1u] != ATT7_FOOTER_2) {
        return 0u;
    }
    if (length == ATT7_COMMAND_LEN) {
        uint8_t command = frame[1];
        uint8_t header_sum = (uint8_t)((ATT7_HEADER_COMMAND + command) & 0xFFu);
        uint8_t table1 = (uint8_t)((command + ATT7_TABLE1_CHECKSUM_OFFSET) & 0xFFu);

        if (is_known_command(command) == 0u) {
            return 0u;
        }
        checksum_ok = (uint8_t)(frame[2] == header_sum || frame[2] == table1);
    } else {
        uint8_t received = frame[ATT7_DATA_CHECKSUM_INDEX];

        checksum_ok = (uint8_t)(received == att7_checksum(frame, ATT7_DATA_CHECKSUM_INDEX,
                                                          ATT7_CHECKSUM_WITH_HEADER) ||
                                received == att7_checksum(frame, ATT7_DATA_CHECKSUM_INDEX,
                                                          ATT7_CHECKSUM_WITHOUT_HEADER));
    }
    if (checksum_ok == 0u) {
        parser->checksum_mismatches++;
        if (config->strict_rx_checksum != 0u) {
            return 0u;
        }
    }

    memset(event, 0, sizeof *event);
    event->checksum_ok = checksum_ok;
    if (length == ATT7_COMMAND_LEN) {
        event->type = ATT7_EVENT_COMMAND;
        event->command = frame[1];
    } else {
        float fields[ATT7_FIELD_COUNT];
        size_t i;

        for (i = 0u; i < ATT7_FIELD_COUNT; i++) {
            fields[i] = get_f32(&frame[1u + 4u * i], config->float_order);
        }
        event->type = ATT7_EVENT_DATA;
        fields_to_sample(fields, &event->sample);
    }
    return 1u;
}

void att7_parser_init(att7_parser_t *parser)
{
    if (parser != NULL) {
        memset(parser, 0, sizeof *parser);
    }
}

uint8_t att7_parser_poll(att7_parser_t *parser, const att7_config_t *config, att7_event_t *event)
{
    if (parser == NULL || config == NULL || event == NULL) {
        return 0u;
    }
    event->type = ATT7_EVENT_NONE;
    while (parser->expected != 0u && parser->len >= parser->expected) {
        if (decode_frame(parser, config, event) != 0u) {
            parser->frames_ok++;
            discard(parser, parser->expected);
            return 1u;
        }
        parser->frames_dropped++;
        discard(parser, 1u);
    }
    return 0u;
}

uint8_t att7_parser_push(att7_parser_t *parser, const att7_config_t *config, uint8_t byte,
                         uint32_t now_ms, att7_event_t *event)
{
    if (parser == NULL || config == NULL || event == NULL) {
        return 0u;
    }
    event->type = ATT7_EVENT_NONE;
    if (parser->len > 0u && (uint32_t)(now_ms - parser->last_byte_ms) > ATT7_FRAME_TIMEOUT_MS) {
        parser->timeouts++;
        parser->len = 0u;
        parser->expected = 0u;
    }
    parser->last_byte_ms = now_ms;
    if (parser->len == 0u) {
        parser->expected = expected_length(byte);
        if (parser->expected == 0u) {
            return 0u; /* noise between frames */
        }
    }
    parser->buf[parser->len] = byte;
    parser->len = (uint8_t)(parser->len + 1u);
    return att7_parser_poll(parser, config, event);
}

/* ---- Session -------------------------------------------------------------- */

void att7_session_init(att7_session_t *session)
{
    if (session != NULL) {
        memset(session, 0, sizeof *session);
        session->mode = ATT7_MODE_NORMAL;
        session->pending = ATT7_MODE_NORMAL;
    }
}

uint8_t att7_session_command(att7_session_t *session, uint8_t command, uint32_t now_ms,
                             uint8_t test_allowed)
{
    if (session == NULL) {
        return 0u;
    }
    if (command == ATT7_CMD_STOP) {
        session->mode = ATT7_MODE_NORMAL;
        session->pending = ATT7_MODE_NORMAL;
        session->sft_data_timeout = 0u;
        return 1u;
    }
    if ((command != ATT7_CMD_START_SMT && command != ATT7_CMD_START_SFT) || test_allowed == 0u) {
        return 0u;
    }
    /* A new start ends any running test first, then waits for the activation delay. */
    session->mode = ATT7_MODE_NORMAL;
    session->pending = (command == ATT7_CMD_START_SMT) ? ATT7_MODE_SMT : ATT7_MODE_SFT;
    session->pending_since_ms = now_ms;
    session->sft_data_timeout = 0u;
    return 1u;
}

void att7_session_note_sft_data(att7_session_t *session, uint32_t now_ms)
{
    if (session == NULL || session->mode != ATT7_MODE_SFT) {
        return;
    }
    session->last_sft_data_ms = now_ms;
    session->sft_data_timeout = 0u;
}

uint8_t att7_session_poll(att7_session_t *session, uint32_t now_ms)
{
    if (session == NULL) {
        return 0u;
    }
    if (session->pending != ATT7_MODE_NORMAL &&
        (uint32_t)(now_ms - session->pending_since_ms) >= ATT7_ACTIVATION_DELAY_MS) {
        session->mode = session->pending;
        session->pending = ATT7_MODE_NORMAL;
        session->next_tx_ms = now_ms;
        session->last_sft_data_ms = now_ms;
        session->sft_data_timeout = 0u;
    }
    if (session->mode == ATT7_MODE_NORMAL) {
        return 0u;
    }
    if (session->mode == ATT7_MODE_SFT &&
        (uint32_t)(now_ms - session->last_sft_data_ms) > ATT7_SFT_DATA_TIMEOUT_MS) {
        session->sft_data_timeout = 1u;
    }
    if ((int32_t)(now_ms - session->next_tx_ms) < 0) {
        return 0u;
    }
    session->next_tx_ms += ATT7_TX_PERIOD_MS;
    if ((int32_t)(now_ms - session->next_tx_ms) >= 0) {
        /* More than a period behind: restart the cadence rather than send a burst. */
        session->next_tx_ms = now_ms + ATT7_TX_PERIOD_MS;
    }
    return 1u;
}

/* ---- Layout checks -------------------------------------------------------- */

size_t att7_sizeof_config(void)
{
    return sizeof(att7_config_t);
}

size_t att7_sizeof_sample(void)
{
    return sizeof(att7_sample_t);
}

size_t att7_sizeof_event(void)
{
    return sizeof(att7_event_t);
}

size_t att7_sizeof_parser(void)
{
    return sizeof(att7_parser_t);
}

size_t att7_sizeof_session(void)
{
    return sizeof(att7_session_t);
}
