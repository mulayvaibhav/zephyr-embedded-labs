#include "vehicle_command_parser.h"

#include <ctype.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

#define DEFAULT_MANUAL_TTL_MS      (400u)
#define DEFAULT_SPEED_LIMIT_PCT    (40u)

static uint32_t g_sequence_id;

static char to_upper_char(char c)
{
    return (char)toupper((unsigned char)c);
}

static void normalize_command_string(const char *in, char *out, size_t out_size)
{
    size_t j = 0u;

    if ((in == NULL) || (out == NULL) || (out_size == 0u)) {
        return;
    }

    for (size_t i = 0u; in[i] != '\0'; i++) {
        char c = in[i];

        if ((c == '\r') || (c == '\n') || (c == ' ') || (c == '\t')) {
            continue;
        }

        if (j < (out_size - 1u)) {
            out[j++] = to_upper_char(c);
        }
    }

    out[j] = '\0';
}

static bool starts_with(const char *s, const char *prefix)
{
    if ((s == NULL) || (prefix == NULL)) {
        return false;
    }

    while (*prefix != '\0') {
        if (*s != *prefix) {
            return false;
        }

        s++;
        prefix++;
    }

    return true;
}

static void fill_common(vehicle_motion_command_t *cmd,
                        vehicle_command_source_t source,
                        uint32_t timestamp_ms)
{
    memset(cmd, 0, sizeof(*cmd));

    cmd->version = VEHICLE_COMMAND_VERSION;
    cmd->source = source;
    cmd->control_mode = VEHICLE_MODE_MANUAL;
    cmd->speed_limit_pct = DEFAULT_SPEED_LIMIT_PCT;
    cmd->ttl_ms = DEFAULT_MANUAL_TTL_MS;
    cmd->sequence_id = ++g_sequence_id;
    cmd->timestamp_ms = timestamp_ms;
}

bool vehicle_parse_ascii_command(const char *raw,
                                 vehicle_command_source_t source,
                                 uint32_t timestamp_ms,
                                 vehicle_motion_command_t *out_cmd)
{
    char cmd_str[32];

    if ((raw == NULL) || (out_cmd == NULL)) {
        return false;
    }

    normalize_command_string(raw, cmd_str, sizeof(cmd_str));

    if (cmd_str[0] == '\0') {
        return false;
    }

    fill_common(out_cmd, source, timestamp_ms);

    /*
     * HC-05 path currently sends:
     *   UP, DWN, LFT, RGT
     *
     * BLE GATT path has also shown:
     *   UP, DOWN, LEFT, RIGHT
     *
     * Accept both vocabularies here.
     */

    if (strcmp(cmd_str, "UP") == 0) {
        out_cmd->command_type = VEHICLE_CMD_MOTION;
        out_cmd->linear_x = +1000;
        out_cmd->angular_z = 0;
        return true;
    }

    if ((strcmp(cmd_str, "DWN") == 0) || (strcmp(cmd_str, "DOWN") == 0)) {
        out_cmd->command_type = VEHICLE_CMD_MOTION;
        out_cmd->linear_x = -1000;
        out_cmd->angular_z = 0;
        return true;
    }

    if ((strcmp(cmd_str, "LFT") == 0) || (strcmp(cmd_str, "LEFT") == 0)) {
        out_cmd->command_type = VEHICLE_CMD_MOTION;
        out_cmd->linear_x = 0;
        out_cmd->angular_z = +1000;
        return true;
    }

    if ((strcmp(cmd_str, "RGT") == 0) || (strcmp(cmd_str, "RIGHT") == 0)) {
        out_cmd->command_type = VEHICLE_CMD_MOTION;
        out_cmd->linear_x = 0;
        out_cmd->angular_z = -1000;
        return true;
    }

    if (strcmp(cmd_str, "C") == 0) {
        out_cmd->command_type = VEHICLE_CMD_STOP;
        out_cmd->linear_x = 0;
        out_cmd->angular_z = 0;
        return true;
    }

    if (strcmp(cmd_str, "D") == 0) {
        out_cmd->command_type = VEHICLE_CMD_EMERGENCY_STOP;
        out_cmd->linear_x = 0;
        out_cmd->angular_z = 0;
        out_cmd->speed_limit_pct = 0;
        return true;
    }

    if (starts_with(cmd_str, "SPEED_")) {
        const char *value_str = &cmd_str[6];
        long value = strtol(value_str, NULL, 10);

        if (value < 0) {
            value = 0;
        }

        if (value > 100) {
            value = 100;
        }

        out_cmd->command_type = VEHICLE_CMD_SET_SPEED;
        out_cmd->speed_limit_pct = (uint8_t)value;
        out_cmd->linear_x = 0;
        out_cmd->angular_z = 0;
        return true;
    }

    return false;
}