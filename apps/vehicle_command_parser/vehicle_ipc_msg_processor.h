#ifndef VEHICLE_IPC_CMD_PROCESSOR_H
#define VEHICLE_IPC_CMD_PROCESSOR_H

#include <stdbool.h>
#include "vehicle_ipc_wire.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    VEHICLE_CMD_PRC_FAIL    = 0,
    VEHICLE_CMD_PRC_SUCCESS = 1,

    VEHICLE_CMD_PRC_INVALID = 255
} vehicle_cmd_prc_val_t;

int vehicle_ipc_cmd_process(void *data,
                            uint32_t len,
                            uint32_t src,
                            vehicle_command_ack_wire_t * ack);

#ifdef __cplusplus
}
#endif

#endif /* VEHICLE_IPC_CMD_PROCESSOR_H */