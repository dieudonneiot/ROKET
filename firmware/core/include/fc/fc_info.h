/*
 * Build information exposed to the Python tools, so they can refuse to run
 * against a flight-core library whose binary interface they do not know.
 */
#ifndef FC_INFO_H
#define FC_INFO_H

#include <stdint.h>

#include "fc/fc_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bump whenever a public struct layout or function signature changes. */
#define FC_ABI_VERSION 1u

FC_API uint32_t fc_abi_version(void);

#ifdef __cplusplus
}
#endif

#endif /* FC_INFO_H */
