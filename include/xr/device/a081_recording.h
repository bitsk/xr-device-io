// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/statvfs.h>

/* Raw archives are an explicit diagnostic operation, never a live-stream
 * requirement. Leave 2 GiB for the operating system and normal shutdown. */
static inline int xr_recording_enabled(void)
{
    const char *value = getenv("XR_DIAGNOSTIC_RAW");
    return value && strcmp(value, "1") == 0;
}
static inline int xr_recording_room(const char *path)
{
    struct statvfs fs;
    return path && statvfs(path, &fs) == 0 &&
           (uint64_t)fs.f_bavail * fs.f_frsize >= UINT64_C(2147483648);
}

static inline int xr_diagnostic_logs_enabled(void)
{
    const char *value = getenv("XR_DIAGNOSTIC_LOGS");
    return value && strcmp(value, "1") == 0;
}
