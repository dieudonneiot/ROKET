/*
 * Export macro for the flight core.
 *
 * The core is linked statically into the firmware and the unit tests, and is
 * also built as a shared library that the Python tools load with ctypes.
 */
#ifndef FC_API_H
#define FC_API_H

#if defined(FC_BUILD_SHARED)
#if defined(_WIN32)
#define FC_API __declspec(dllexport)
#else
#define FC_API __attribute__((visibility("default")))
#endif
#else
#define FC_API
#endif

#endif /* FC_API_H */
