#include <zephyr/kernel.h>
#include <arpa/inet.h>

#include "vehicle_ipc_msg_processor.h"
#include "vehicle_control_manager.h"

#include <ctype.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

static uint32_t vehicle_get_time_ms(void)
{
    return k_uptime_get_32();
}

static void populate_ack(uint32_t dst,
                                 uint32_t sequence_id,
                                 uint8_t status,
                                 uint16_t error_code,
                                 vehicle_command_ack_wire_t * ack)
{
    ack->magic = VEHICLE_ACK_MAGIC;
    ack->version = VEHICLE_WIRE_VERSION;
    ack->status = status;
    ack->error_code = error_code;
    ack->sequence_id = sequence_id;
    ack->timestamp_ms = vehicle_get_time_ms();
}

static void convert_wire_to_vehicle_command(
    const vehicle_motion_command_wire_t *wire,
    vehicle_motion_command_t *cmd)
{
    memset(cmd, 0, sizeof(*cmd));

    cmd->version = wire->version;
    cmd->source = wire->source;
    cmd->command_type = wire->command_type;
    cmd->control_mode = wire->control_mode;

    cmd->linear_x = wire->linear_x;
    cmd->angular_z = wire->angular_z;

    cmd->speed_limit_pct = wire->speed_limit_pct;
    cmd->ttl_ms = wire->ttl_ms;

    cmd->sequence_id = wire->sequence_id;
    cmd->timestamp_ms = wire->timestamp_ms;
}

int vehicle_ipc_cmd_process(void *data,
                            uint32_t len,
                            uint32_t src,
                            vehicle_command_ack_wire_t * ack)
{
    if( data == NULL || ack == NULL ) {
        populate_ack(src, 0, VEHICLE_ACK_STATUS_REJECTED, (uint16_t)len, ack);
        return VEHICLE_CMD_PRC_FAIL;
    }

    if (len != sizeof(vehicle_motion_command_wire_t)) {
        populate_ack(src, 0,
                         VEHICLE_ACK_STATUS_INVALID_LENGTH,
                         (uint16_t)len, ack);
        return VEHICLE_CMD_PRC_FAIL;
    }

    vehicle_motion_command_wire_t cmd;
    memcpy(&cmd, data, len);

    cmd.magic = ntohl(cmd.magic);
    cmd.linear_x  = (int16_t)ntohs((uint16_t)cmd.linear_x);
    cmd.angular_z = (int16_t)ntohs((uint16_t)cmd.angular_z);

    cmd.ttl_ms = ntohs(cmd.ttl_ms);
    cmd.sequence_id = ntohl(cmd.sequence_id);
    cmd.timestamp_ms = k_uptime_get_32();

    if (cmd.magic != VEHICLE_CMD_MAGIC) {
        populate_ack(src, cmd.sequence_id,
                         VEHICLE_ACK_STATUS_INVALID_MAGIC,
                         0, ack);
        return VEHICLE_CMD_PRC_FAIL;
    }

    if (cmd.version != VEHICLE_WIRE_VERSION) {
        populate_ack(src, cmd.sequence_id,
                         VEHICLE_ACK_STATUS_INVALID_VERSION,
                         cmd.version, ack);
        return VEHICLE_CMD_PRC_FAIL;
    }

    if (cmd.speed_limit_pct > 100) {
        populate_ack(src, cmd.sequence_id,
                         VEHICLE_ACK_STATUS_INVALID_RANGE,
                         cmd.speed_limit_pct, ack);
        return VEHICLE_CMD_PRC_FAIL;
    }

    if (cmd.linear_x < -1000 || cmd.linear_x > 1000 ||
        cmd.angular_z < -1000 || cmd.angular_z > 1000) {

        populate_ack(src, cmd.sequence_id,
                         VEHICLE_ACK_STATUS_INVALID_RANGE,
                         0, ack);
        
        return VEHICLE_CMD_PRC_FAIL;
    }

    vehicle_motion_command_t vehicle_cmd;
    convert_wire_to_vehicle_command(&cmd, &vehicle_cmd);

    int ret = vehicle_control_manager_submit_command(&vehicle_cmd);
    if (ret == 0) {
        populate_ack(src, cmd.sequence_id,
                     VEHICLE_ACK_STATUS_OK,
                     0, 
                     ack);
    } else {
        populate_ack(src, cmd.sequence_id,
                        VEHICLE_ACK_STATUS_REJECTED,
                        (uint16_t)(-ret), ack);
        return VEHICLE_CMD_PRC_FAIL;
    }

    return VEHICLE_CMD_PRC_SUCCESS;
}

