#include <string.h>
#include <zephyr/kernel.h>
#include <openamp/open_amp.h>

#include "vehicle_ipc_wire.h"
#include "rpmsg_processor.h"

static struct rpmsg_endpoint vehicle_cmd_ept;

static uint32_t vehicle_get_time_ms(void)
{
    return k_uptime_get_32();
}

static void vehicle_send_ack(struct rpmsg_endpoint *ept,
                             uint32_t dst,
                             uint32_t sequence_id,
                             uint8_t status,
                             uint16_t error_code)
{
    vehicle_command_ack_wire_t ack = {
        .magic = VEHICLE_ACK_MAGIC,
        .version = VEHICLE_WIRE_VERSION,
        .status = status,
        .error_code = error_code,
        .sequence_id = sequence_id,
        .timestamp_ms = vehicle_get_time_ms(),
    };

    /*
     * Send ACK back to the Linux endpoint that sent the command.
     */
    (void)rpmsg_sendto(ept, &ack, sizeof(ack), dst);
}

static int vehicle_cmd_rpmsg_cb(struct rpmsg_endpoint *ept,
                                void *data,
                                size_t len,
                                uint32_t src,
                                void *priv)
{
    ARG_UNUSED(priv);

    if (len != sizeof(vehicle_motion_command_wire_t)) {
        vehicle_send_ack(ept, src, 0,
                         VEHICLE_ACK_STATUS_INVALID_LENGTH,
                         (uint16_t)len);
        return RPMSG_SUCCESS;
    }

    vehicle_motion_command_wire_t cmd;
    memcpy(&cmd, data, sizeof(cmd));

    if (cmd.magic != VEHICLE_CMD_MAGIC) {
        vehicle_send_ack(ept, src, cmd.sequence_id,
                         VEHICLE_ACK_STATUS_INVALID_MAGIC,
                         0);
        return RPMSG_SUCCESS;
    }

    if (cmd.version != VEHICLE_WIRE_VERSION) {
        vehicle_send_ack(ept, src, cmd.sequence_id,
                         VEHICLE_ACK_STATUS_INVALID_VERSION,
                         cmd.version);
        return RPMSG_SUCCESS;
    }

    if (cmd.speed_limit_pct > 100) {
        vehicle_send_ack(ept, src, cmd.sequence_id,
                         VEHICLE_ACK_STATUS_INVALID_RANGE,
                         cmd.speed_limit_pct);
        return RPMSG_SUCCESS;
    }

    if (cmd.linear_x < -1000 || cmd.linear_x > 1000 ||
        cmd.angular_z < -1000 || cmd.angular_z > 1000) {
        vehicle_send_ack(ept, src, cmd.sequence_id,
                         VEHICLE_ACK_STATUS_INVALID_RANGE,
                         0);
        return RPMSG_SUCCESS;
    }

    /*
     * At this point M33 has received and validated the binary command.
     *
     * Later:
     *   convert cmd -> vehicle_motion_command_t
     *   call vehicle_control_manager_submit_command(...)
     */

    vehicle_send_ack(ept, src, cmd.sequence_id,
                     VEHICLE_ACK_STATUS_OK,
                     0);

    return RPMSG_SUCCESS;
}

int vehicle_cmd_rpmsg_start(struct rpmsg_device *rpdev)
{
    return rpmsg_create_ept(&vehicle_cmd_ept,
                            rpdev,
                            "rpmsg-raw",
                            RPMSG_ADDR_ANY,
                            RPMSG_ADDR_ANY,
                            vehicle_cmd_rpmsg_cb,
                            NULL);
}