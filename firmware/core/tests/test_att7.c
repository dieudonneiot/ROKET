/*
 * Unit tests for the Attachment-7 codec, parser and session (findings log F-20 to F-29).
 * Golden byte vectors were computed independently with Python's struct module.
 */
#include <math.h>
#include <string.h>

#include "fc/att7.h"
#include "fc/fc_info.h"
#include "unity.h"

void setUp(void)
{
}

void tearDown(void)
{
}

static att7_config_t default_config(void)
{
    att7_config_t config;

    att7_config_default(&config);
    return config;
}

/* Every value is exactly representable, so 2-decimal rounding leaves it unchanged. */
static const att7_sample_t golden_sample = {1234.5f, 1013.25f, 98.0f, -0.5f,
                                            9.75f,   1.25f,    85.5f, -180.0f};

static const uint8_t golden_big_endian[ATT7_DATA_LEN] = {
    0xAB, 0x44, 0x9A, 0x50, 0x00, 0x44, 0x7D, 0x50, 0x00, 0x42, 0xC4, 0x00,
    0x00, 0xBF, 0x00, 0x00, 0x00, 0x41, 0x1C, 0x00, 0x00, 0x3F, 0xA0, 0x00,
    0x00, 0x42, 0xAB, 0x00, 0x00, 0xC3, 0x34, 0x00, 0x00, 0xCF, 0x0D, 0x0A};

static const uint8_t golden_little_endian[ATT7_DATA_LEN] = {
    0xAB, 0x00, 0x50, 0x9A, 0x44, 0x00, 0x50, 0x7D, 0x44, 0x00, 0x00, 0xC4,
    0x42, 0x00, 0x00, 0x00, 0xBF, 0x00, 0x00, 0x1C, 0x41, 0x00, 0x00, 0xA0,
    0x3F, 0x00, 0x00, 0xAB, 0x42, 0x00, 0x00, 0x34, 0xC3, 0xCF, 0x0D, 0x0A};

/* Feeds bytes one at a time at the same timestamp; returns the number of events. */
static unsigned feed(att7_parser_t *parser, const att7_config_t *config, const uint8_t *bytes,
                     size_t count, uint32_t now_ms, att7_event_t *last_event)
{
    unsigned events = 0u;
    size_t i;

    for (i = 0u; i < count; i++) {
        att7_event_t event;

        if (att7_parser_push(parser, config, bytes[i], now_ms, &event) != 0u) {
            events++;
            *last_event = event;
            while (att7_parser_poll(parser, config, &event) != 0u) {
                events++;
                *last_event = event;
            }
        }
    }
    return events;
}

static void assert_sample_equal(const att7_sample_t *expected, const att7_sample_t *actual)
{
    TEST_ASSERT_EQUAL_FLOAT(expected->altitude_m, actual->altitude_m);
    TEST_ASSERT_EQUAL_FLOAT(expected->pressure_mbar, actual->pressure_mbar);
    TEST_ASSERT_EQUAL_FLOAT(expected->accel_x_mps2, actual->accel_x_mps2);
    TEST_ASSERT_EQUAL_FLOAT(expected->accel_y_mps2, actual->accel_y_mps2);
    TEST_ASSERT_EQUAL_FLOAT(expected->accel_z_mps2, actual->accel_z_mps2);
    TEST_ASSERT_EQUAL_FLOAT(expected->angle_x_deg, actual->angle_x_deg);
    TEST_ASSERT_EQUAL_FLOAT(expected->angle_y_deg, actual->angle_y_deg);
    TEST_ASSERT_EQUAL_FLOAT(expected->angle_z_deg, actual->angle_z_deg);
}

/* ---- Configuration and codec ---------------------------------------------- */

static void test_defaults_follow_f29(void)
{
    att7_config_t config = default_config();

    TEST_ASSERT_EQUAL_UINT8(ATT7_CHECKSUM_WITH_HEADER, config.tx_checksum);
    TEST_ASSERT_EQUAL_UINT8(ATT7_FLOAT_BIG_ENDIAN, config.float_order);
    TEST_ASSERT_EQUAL_UINT8(1u, config.round_2dp);
    TEST_ASSERT_EQUAL_UINT8(0u, config.strict_rx_checksum);
    TEST_ASSERT_EQUAL_UINT8(1u, config.status_low_first);
}

static void test_checksum_with_and_without_header(void)
{
    const uint8_t frame[] = {0xAA, 0x14, 0x00};

    TEST_ASSERT_EQUAL_HEX8(0xBE, att7_checksum(frame, 3u, ATT7_CHECKSUM_WITH_HEADER));
    TEST_ASSERT_EQUAL_HEX8(0x14, att7_checksum(frame, 3u, ATT7_CHECKSUM_WITHOUT_HEADER));
    TEST_ASSERT_EQUAL_HEX8(0x00, att7_checksum(NULL, 3u, ATT7_CHECKSUM_WITH_HEADER));
}

static void test_encode_command_both_checksum_styles(void)
{
    const uint8_t smt_header_sum[] = {0xAA, 0x20, 0xCA, 0x0D, 0x0A};
    const uint8_t sft_header_sum[] = {0xAA, 0x22, 0xCC, 0x0D, 0x0A};
    const uint8_t stop_header_sum[] = {0xAA, 0x24, 0xCE, 0x0D, 0x0A};
    const uint8_t smt_table1[] = {0xAA, 0x20, 0x8C, 0x0D, 0x0A};
    const uint8_t sft_table1[] = {0xAA, 0x22, 0x8E, 0x0D, 0x0A};
    const uint8_t stop_table1[] = {0xAA, 0x24, 0x90, 0x0D, 0x0A};
    uint8_t out[ATT7_COMMAND_LEN];

    TEST_ASSERT_EQUAL_UINT(
        ATT7_COMMAND_LEN,
        att7_encode_command(ATT7_CMD_START_SMT, ATT7_COMMAND_CHECKSUM_HEADER_SUM, out));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(smt_header_sum, out, ATT7_COMMAND_LEN);
    att7_encode_command(ATT7_CMD_START_SFT, ATT7_COMMAND_CHECKSUM_HEADER_SUM, out);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(sft_header_sum, out, ATT7_COMMAND_LEN);
    att7_encode_command(ATT7_CMD_STOP, ATT7_COMMAND_CHECKSUM_HEADER_SUM, out);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(stop_header_sum, out, ATT7_COMMAND_LEN);

    att7_encode_command(ATT7_CMD_START_SMT, ATT7_COMMAND_CHECKSUM_TABLE1, out);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(smt_table1, out, ATT7_COMMAND_LEN);
    att7_encode_command(ATT7_CMD_START_SFT, ATT7_COMMAND_CHECKSUM_TABLE1, out);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(sft_table1, out, ATT7_COMMAND_LEN);
    att7_encode_command(ATT7_CMD_STOP, ATT7_COMMAND_CHECKSUM_TABLE1, out);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(stop_table1, out, ATT7_COMMAND_LEN);

    TEST_ASSERT_EQUAL_UINT(0u, att7_encode_command(ATT7_CMD_STOP, 0u, NULL));
}

static void test_encode_data_big_endian_golden(void)
{
    att7_config_t config = default_config();
    uint8_t out[ATT7_DATA_LEN];

    TEST_ASSERT_EQUAL_UINT(ATT7_DATA_LEN, att7_encode_data(&config, &golden_sample, out));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(golden_big_endian, out, ATT7_DATA_LEN);
}

static void test_encode_data_little_endian_golden(void)
{
    att7_config_t config = default_config();
    uint8_t out[ATT7_DATA_LEN];

    config.float_order = ATT7_FLOAT_LITTLE_ENDIAN;
    att7_encode_data(&config, &golden_sample, out);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(golden_little_endian, out, ATT7_DATA_LEN);
}

static void test_encode_data_checksum_without_header(void)
{
    att7_config_t config = default_config();
    uint8_t out[ATT7_DATA_LEN];

    config.tx_checksum = ATT7_CHECKSUM_WITHOUT_HEADER;
    att7_encode_data(&config, &golden_sample, out);
    TEST_ASSERT_EQUAL_HEX8(0x24, out[ATT7_DATA_CHECKSUM_INDEX]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(golden_big_endian, out, ATT7_DATA_CHECKSUM_INDEX);
}

static void test_round_2dp(void)
{
    TEST_ASSERT_EQUAL_FLOAT(1.23f, att7_round_2dp(1.23456f));
    TEST_ASSERT_EQUAL_FLOAT(-1.24f, att7_round_2dp(-1.235f));
    TEST_ASSERT_EQUAL_FLOAT(2.68f, att7_round_2dp(2.675f));
    TEST_ASSERT_EQUAL_FLOAT(0.01f, att7_round_2dp(0.005f));
    TEST_ASSERT_EQUAL_FLOAT(1013.26f, att7_round_2dp(1013.256f));

    /* A value that rounds to zero is sent as +0.0, never -0.0. */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, att7_round_2dp(-0.004999f));
    TEST_ASSERT_FALSE(signbit(att7_round_2dp(-0.004999f)));

    /* Non-finite and very large values pass through unchanged. */
    TEST_ASSERT_TRUE(isnan(att7_round_2dp(NAN)));
    TEST_ASSERT_TRUE(isinf(att7_round_2dp(INFINITY)));
    TEST_ASSERT_EQUAL_FLOAT(3.0e8f, att7_round_2dp(3.0e8f));
}

static void test_encode_data_rounds_when_enabled(void)
{
    att7_config_t config = default_config();
    att7_sample_t sample = golden_sample;
    att7_parser_t parser;
    att7_event_t event;
    uint8_t out[ATT7_DATA_LEN];

    sample.altitude_m = 1234.5678f;
    att7_encode_data(&config, &sample, out);
    att7_parser_init(&parser);
    TEST_ASSERT_EQUAL_UINT(1u, feed(&parser, &config, out, sizeof out, 0u, &event));
    TEST_ASSERT_EQUAL_FLOAT(1234.57f, event.sample.altitude_m);

    config.round_2dp = 0u;
    att7_encode_data(&config, &sample, out);
    att7_parser_init(&parser);
    feed(&parser, &config, out, sizeof out, 0u, &event);
    TEST_ASSERT_EQUAL_FLOAT(1234.5678f, event.sample.altitude_m);
}

static void test_encode_status(void)
{
    const uint8_t example[] = {0xAA, 0x14, 0x00, 0xBE, 0x0D, 0x0A};
    const uint8_t high_first[] = {0xAA, 0x00, 0x14, 0xBE, 0x0D, 0x0A};
    att7_config_t config = default_config();
    uint8_t out[ATT7_STATUS_LEN];

    TEST_ASSERT_EQUAL_UINT(
        ATT7_STATUS_LEN,
        att7_encode_status(&config, ATT7_STATUS_MIN_ALTITUDE | ATT7_STATUS_DESCENDING, out));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(example, out, ATT7_STATUS_LEN);

    config.tx_checksum = ATT7_CHECKSUM_WITHOUT_HEADER;
    att7_encode_status(&config, 0x0014u, out);
    TEST_ASSERT_EQUAL_HEX8(0x14, out[ATT7_STATUS_CHECKSUM_INDEX]);

    config.tx_checksum = ATT7_CHECKSUM_WITH_HEADER;
    config.status_low_first = 0u;
    att7_encode_status(&config, 0x0014u, out);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(high_first, out, ATT7_STATUS_LEN);
}

/* ---- Parser --------------------------------------------------------------- */

static void test_parser_accepts_both_command_checksum_families(void)
{
    const uint8_t header_sum[] = {0xAA, 0x22, 0xCC, 0x0D, 0x0A};
    const uint8_t table1[] = {0xAA, 0x22, 0x8E, 0x0D, 0x0A};
    att7_config_t config = default_config();
    att7_parser_t parser;
    att7_event_t event;

    config.strict_rx_checksum = 1u;
    att7_parser_init(&parser);

    TEST_ASSERT_EQUAL_UINT(1u, feed(&parser, &config, header_sum, sizeof header_sum, 0u, &event));
    TEST_ASSERT_EQUAL_UINT8(ATT7_EVENT_COMMAND, event.type);
    TEST_ASSERT_EQUAL_HEX8(ATT7_CMD_START_SFT, event.command);
    TEST_ASSERT_EQUAL_UINT8(1u, event.checksum_ok);

    TEST_ASSERT_EQUAL_UINT(1u, feed(&parser, &config, table1, sizeof table1, 10u, &event));
    TEST_ASSERT_EQUAL_UINT8(1u, event.checksum_ok);
    TEST_ASSERT_EQUAL_UINT32(2u, parser.frames_ok);
    TEST_ASSERT_EQUAL_UINT32(0u, parser.checksum_mismatches);
}

static void test_parser_lenient_mode_counts_bad_checksum(void)
{
    const uint8_t bad[] = {0xAA, 0x24, 0x00, 0x0D, 0x0A};
    att7_config_t config = default_config();
    att7_parser_t parser;
    att7_event_t event;

    att7_parser_init(&parser);
    TEST_ASSERT_EQUAL_UINT(1u, feed(&parser, &config, bad, sizeof bad, 0u, &event));
    TEST_ASSERT_EQUAL_HEX8(ATT7_CMD_STOP, event.command);
    TEST_ASSERT_EQUAL_UINT8(0u, event.checksum_ok);
    TEST_ASSERT_EQUAL_UINT32(1u, parser.checksum_mismatches);
}

static void test_parser_strict_mode_drops_bad_checksum(void)
{
    const uint8_t bad[] = {0xAA, 0x24, 0x00, 0x0D, 0x0A};
    att7_config_t config = default_config();
    att7_parser_t parser;
    att7_event_t event;

    config.strict_rx_checksum = 1u;
    att7_parser_init(&parser);
    TEST_ASSERT_EQUAL_UINT(0u, feed(&parser, &config, bad, sizeof bad, 0u, &event));
    TEST_ASSERT_EQUAL_UINT32(1u, parser.checksum_mismatches);
    TEST_ASSERT_EQUAL_UINT32(1u, parser.frames_dropped);
}

static void test_parser_data_round_trip_both_byte_orders(void)
{
    att7_config_t config = default_config();
    att7_parser_t parser;
    att7_event_t event;

    att7_parser_init(&parser);
    TEST_ASSERT_EQUAL_UINT(1u,
                           feed(&parser, &config, golden_big_endian, ATT7_DATA_LEN, 0u, &event));
    TEST_ASSERT_EQUAL_UINT8(ATT7_EVENT_DATA, event.type);
    TEST_ASSERT_EQUAL_UINT8(1u, event.checksum_ok);
    assert_sample_equal(&golden_sample, &event.sample);

    config.float_order = ATT7_FLOAT_LITTLE_ENDIAN;
    TEST_ASSERT_EQUAL_UINT(
        1u, feed(&parser, &config, golden_little_endian, ATT7_DATA_LEN, 10u, &event));
    assert_sample_equal(&golden_sample, &event.sample);
}

static void test_parser_accepts_data_checksum_without_header(void)
{
    att7_config_t config = default_config();
    att7_parser_t parser;
    att7_event_t event;
    uint8_t frame[ATT7_DATA_LEN];

    memcpy(frame, golden_big_endian, sizeof frame);
    frame[ATT7_DATA_CHECKSUM_INDEX] = 0x24; /* sum of bytes 1..32 */
    config.strict_rx_checksum = 1u;
    att7_parser_init(&parser);
    TEST_ASSERT_EQUAL_UINT(1u, feed(&parser, &config, frame, sizeof frame, 0u, &event));
    TEST_ASSERT_EQUAL_UINT8(1u, event.checksum_ok);
}

static void test_parser_skips_noise_and_handles_split_frames(void)
{
    const uint8_t noise[] = {0x00, 0x13, 0xFF, 0x0D, 0x0A};
    const uint8_t stop[] = {0xAA, 0x24, 0xCE, 0x0D, 0x0A};
    att7_config_t config = default_config();
    att7_parser_t parser;
    att7_event_t event;

    att7_parser_init(&parser);
    TEST_ASSERT_EQUAL_UINT(0u, feed(&parser, &config, noise, sizeof noise, 0u, &event));
    TEST_ASSERT_EQUAL_UINT(0u, feed(&parser, &config, stop, 2u, 5u, &event));
    TEST_ASSERT_EQUAL_UINT(1u, feed(&parser, &config, &stop[2], 3u, 20u, &event));
    TEST_ASSERT_EQUAL_HEX8(ATT7_CMD_STOP, event.command);
}

static void test_parser_timeout_drops_partial_frame(void)
{
    const uint8_t stop[] = {0xAA, 0x24, 0xCE, 0x0D, 0x0A};
    att7_config_t config = default_config();
    att7_parser_t parser;
    att7_event_t event;

    att7_parser_init(&parser);
    feed(&parser, &config, golden_big_endian, 10u, 0u, &event); /* truncated data frame */
    TEST_ASSERT_EQUAL_UINT(
        1u, feed(&parser, &config, stop, sizeof stop, ATT7_FRAME_TIMEOUT_MS + 1u, &event));
    TEST_ASSERT_EQUAL_HEX8(ATT7_CMD_STOP, event.command);
    TEST_ASSERT_EQUAL_UINT32(1u, parser.timeouts);
}

static void test_parser_resyncs_after_bad_footer(void)
{
    /* A corrupted data frame that hides a complete Start SFT command. */
    uint8_t stream[ATT7_DATA_LEN];
    const uint8_t start_sft[] = {0xAA, 0x22, 0xCC, 0x0D, 0x0A};
    att7_config_t config = default_config();
    att7_parser_t parser;
    att7_event_t event;

    memset(stream, 0x11, sizeof stream);
    stream[0] = ATT7_HEADER_DATA;
    memcpy(&stream[5], start_sft, sizeof start_sft);

    att7_parser_init(&parser);
    TEST_ASSERT_EQUAL_UINT(1u, feed(&parser, &config, stream, sizeof stream, 0u, &event));
    TEST_ASSERT_EQUAL_UINT8(ATT7_EVENT_COMMAND, event.type);
    TEST_ASSERT_EQUAL_HEX8(ATT7_CMD_START_SFT, event.command);
    TEST_ASSERT_EQUAL_UINT32(1u, parser.frames_dropped);
}

static void test_parser_rejects_unknown_command(void)
{
    const uint8_t unknown[] = {0xAA, 0x99, 0x43, 0x0D, 0x0A};
    att7_config_t config = default_config();
    att7_parser_t parser;
    att7_event_t event;

    att7_parser_init(&parser);
    TEST_ASSERT_EQUAL_UINT(0u, feed(&parser, &config, unknown, sizeof unknown, 0u, &event));
    TEST_ASSERT_EQUAL_UINT32(1u, parser.frames_dropped);
}

/* ---- Session -------------------------------------------------------------- */

static void test_session_activation_delay_and_cadence(void)
{
    att7_session_t session;

    att7_session_init(&session);
    TEST_ASSERT_EQUAL_UINT8(1u, att7_session_command(&session, ATT7_CMD_START_SMT, 0u, 1u));
    TEST_ASSERT_EQUAL_UINT8(0u, att7_session_poll(&session, 999u));
    TEST_ASSERT_EQUAL_UINT8(ATT7_MODE_NORMAL, session.mode);

    TEST_ASSERT_EQUAL_UINT8(1u, att7_session_poll(&session, 1000u));
    TEST_ASSERT_EQUAL_UINT8(ATT7_MODE_SMT, session.mode);
    TEST_ASSERT_EQUAL_UINT8(0u, att7_session_poll(&session, 1050u));
    TEST_ASSERT_EQUAL_UINT8(1u, att7_session_poll(&session, 1100u));
    TEST_ASSERT_EQUAL_UINT8(1u, att7_session_poll(&session, 1200u));

    /* After a stall the cadence restarts instead of sending a burst. */
    TEST_ASSERT_EQUAL_UINT8(1u, att7_session_poll(&session, 1550u));
    TEST_ASSERT_EQUAL_UINT8(0u, att7_session_poll(&session, 1600u));
    TEST_ASSERT_EQUAL_UINT8(1u, att7_session_poll(&session, 1650u));

    TEST_ASSERT_EQUAL_UINT8(1u, att7_session_command(&session, ATT7_CMD_STOP, 1700u, 0u));
    TEST_ASSERT_EQUAL_UINT8(ATT7_MODE_NORMAL, session.mode);
    TEST_ASSERT_EQUAL_UINT8(0u, att7_session_poll(&session, 1800u));
}

static void test_session_refuses_tests_unless_allowed(void)
{
    att7_session_t session;

    att7_session_init(&session);
    TEST_ASSERT_EQUAL_UINT8(0u, att7_session_command(&session, ATT7_CMD_START_SFT, 0u, 0u));
    TEST_ASSERT_EQUAL_UINT8(0u, att7_session_poll(&session, 2000u));
    TEST_ASSERT_EQUAL_UINT8(ATT7_MODE_NORMAL, session.mode);
    TEST_ASSERT_EQUAL_UINT8(0u, att7_session_command(&session, 0x99u, 0u, 1u));
    TEST_ASSERT_EQUAL_UINT8(1u, att7_session_command(&session, ATT7_CMD_STOP, 0u, 0u));
}

static void test_session_restart_switches_mode_after_delay(void)
{
    att7_session_t session;

    att7_session_init(&session);
    att7_session_command(&session, ATT7_CMD_START_SMT, 0u, 1u);
    att7_session_poll(&session, 1000u);
    TEST_ASSERT_EQUAL_UINT8(ATT7_MODE_SMT, session.mode);

    att7_session_command(&session, ATT7_CMD_START_SFT, 1500u, 1u);
    TEST_ASSERT_EQUAL_UINT8(ATT7_MODE_NORMAL, session.mode);
    TEST_ASSERT_EQUAL_UINT8(0u, att7_session_poll(&session, 2499u));
    TEST_ASSERT_EQUAL_UINT8(1u, att7_session_poll(&session, 2500u));
    TEST_ASSERT_EQUAL_UINT8(ATT7_MODE_SFT, session.mode);
}

static void test_session_flags_missing_sft_data(void)
{
    att7_session_t session;

    att7_session_init(&session);
    att7_session_command(&session, ATT7_CMD_START_SFT, 0u, 1u);
    att7_session_poll(&session, 1000u);
    att7_session_note_sft_data(&session, 1100u);

    att7_session_poll(&session, 2100u);
    TEST_ASSERT_EQUAL_UINT8(0u, session.sft_data_timeout);
    att7_session_poll(&session, 2101u);
    TEST_ASSERT_EQUAL_UINT8(1u, session.sft_data_timeout);

    att7_session_note_sft_data(&session, 2200u);
    TEST_ASSERT_EQUAL_UINT8(0u, session.sft_data_timeout);
}

/* ---- Binary interface ----------------------------------------------------- */

static void test_layout_for_python_bindings(void)
{
    TEST_ASSERT_EQUAL_UINT32(2u, fc_abi_version());
    TEST_ASSERT_EQUAL_UINT(5u, att7_sizeof_config());
    TEST_ASSERT_EQUAL_UINT(32u, att7_sizeof_sample());
    TEST_ASSERT_EQUAL_UINT(36u, att7_sizeof_event());
    TEST_ASSERT_EQUAL_UINT(60u, att7_sizeof_parser());
    TEST_ASSERT_EQUAL_UINT(16u, att7_sizeof_session());
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_defaults_follow_f29);
    RUN_TEST(test_checksum_with_and_without_header);
    RUN_TEST(test_encode_command_both_checksum_styles);
    RUN_TEST(test_encode_data_big_endian_golden);
    RUN_TEST(test_encode_data_little_endian_golden);
    RUN_TEST(test_encode_data_checksum_without_header);
    RUN_TEST(test_round_2dp);
    RUN_TEST(test_encode_data_rounds_when_enabled);
    RUN_TEST(test_encode_status);
    RUN_TEST(test_parser_accepts_both_command_checksum_families);
    RUN_TEST(test_parser_lenient_mode_counts_bad_checksum);
    RUN_TEST(test_parser_strict_mode_drops_bad_checksum);
    RUN_TEST(test_parser_data_round_trip_both_byte_orders);
    RUN_TEST(test_parser_accepts_data_checksum_without_header);
    RUN_TEST(test_parser_skips_noise_and_handles_split_frames);
    RUN_TEST(test_parser_timeout_drops_partial_frame);
    RUN_TEST(test_parser_resyncs_after_bad_footer);
    RUN_TEST(test_parser_rejects_unknown_command);
    RUN_TEST(test_session_activation_delay_and_cadence);
    RUN_TEST(test_session_refuses_tests_unless_allowed);
    RUN_TEST(test_session_restart_switches_mode_after_delay);
    RUN_TEST(test_session_flags_missing_sft_data);
    RUN_TEST(test_layout_for_python_bindings);
    return UNITY_END();
}
