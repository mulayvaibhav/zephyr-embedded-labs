#ifndef VEHICLE_COMMAND_PARSER_H
#define VEHICLE_COMMAND_PARSER_H

#include <stdbool.h>
#include "vehicle_command.h"

#ifdef __cplusplus
extern "C" {
#endif

bool vehicle_parse_ascii_command(const char *raw,
                                 vehicle_command_source_t source,
                                 uint32_t timestamp_ms,
                                 vehicle_motion_command_t *out_cmd);

#ifdef __cplusplus
}
#endif

#endif /* VEHICLE_COMMAND_PARSER_H */