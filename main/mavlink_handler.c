#include "mavlink_handler.h"
#include "config.h"
#include "params.h"
#include "pwm_output.h"
#include "wifi_link.h"

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_SEND_UART_BYTES mavlink_send_uart_bytes

static mavlink_system_t mavlink_system = { SYSTEM_ID, COMPONENT_ID };

// forward declare send function
static void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *buf, uint16_t len);

#include "mavlink/common/mavlink.h"

static mavlink_message_t rx_msg[2];
static mavlink_status_t rx_status[2];

// USB CDC send callback (set from main)
static void (*usb_send_fn)(const uint8_t *buf, int len) = NULL;

void mavlink_handler_set_usb_send(void (*fn)(const uint8_t *, int));

void mavlink_handler_set_usb_send(void (*fn)(const uint8_t *, int)) {
    usb_send_fn = fn;
}

static void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *buf, uint16_t len) {
    if (chan == MAVLINK_COMM_0 && usb_send_fn) {
        usb_send_fn(buf, len);
    } else if (chan == MAVLINK_COMM_1) {
        wifi_link_send(buf, len);
    }
}

static void handle_command_long(mavlink_message_t *msg) {
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(msg, &cmd);

    uint8_t result = MAV_RESULT_UNSUPPORTED;

    switch ((uint16_t)cmd.command) {
        case MAV_CMD_COMPONENT_ARM_DISARM:
            // param1: 1=arm, 0=disarm
            result = MAV_RESULT_ACCEPTED;
            break;
        case MAV_CMD_REQUEST_MESSAGE:
            result = MAV_RESULT_ACCEPTED;
            break;
        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            result = MAV_RESULT_ACCEPTED;
            break;
        default:
            break;
    }

    mavlink_message_t ack;
    mavlink_msg_command_ack_pack(SYSTEM_ID, COMPONENT_ID, &ack,
        cmd.command, result, 0, 0, msg->sysid, msg->compid);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &ack);
    mavlink_send_uart_bytes(MAVLINK_COMM_0, buf, len);
    mavlink_send_uart_bytes(MAVLINK_COMM_1, buf, len);
}

static void handle_param_request_list(mavlink_message_t *msg, int link_id) {
    int count = params_count();
    for (int i = 0; i < count; i++) {
        param_entry_t *p = params_get_by_index(i);
        if (!p) continue;
        mavlink_message_t m;
        mavlink_msg_param_value_pack(SYSTEM_ID, COMPONENT_ID, &m,
            p->name, p->value, p->type, count, i);
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &m);
        mavlink_send_uart_bytes(link_id, buf, len);
    }
}

static void handle_param_request_read(mavlink_message_t *msg, int link_id) {
    mavlink_param_request_read_t req;
    mavlink_msg_param_request_read_decode(msg, &req);

    param_entry_t *p = NULL;
    if (req.param_index >= 0) {
        p = params_get_by_index(req.param_index);
    } else {
        char name[17] = {0};
        memcpy(name, req.param_id, 16);
        p = params_get_by_name(name);
    }
    if (!p) return;

    int idx = 0;
    for (int i = 0; i < params_count(); i++) {
        if (params_get_by_index(i) == p) { idx = i; break; }
    }

    mavlink_message_t m;
    mavlink_msg_param_value_pack(SYSTEM_ID, COMPONENT_ID, &m,
        p->name, p->value, p->type, params_count(), idx);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &m);
    mavlink_send_uart_bytes(link_id, buf, len);
}

static void handle_param_set(mavlink_message_t *msg, int link_id) {
    mavlink_param_set_t set;
    mavlink_msg_param_set_decode(msg, &set);

    char name[17] = {0};
    memcpy(name, set.param_id, 16);
    params_set(name, set.param_value);
    params_save();

    param_entry_t *p = params_get_by_name(name);
    if (!p) return;

    int idx = 0;
    for (int i = 0; i < params_count(); i++) {
        if (params_get_by_index(i) == p) { idx = i; break; }
    }

    mavlink_message_t m;
    mavlink_msg_param_value_pack(SYSTEM_ID, COMPONENT_ID, &m,
        p->name, p->value, p->type, params_count(), idx);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &m);
    mavlink_send_uart_bytes(link_id, buf, len);
}

static void handle_rc_override(mavlink_message_t *msg);
static rover_state_t *g_state = NULL;

void mavlink_handler_set_state(rover_state_t *state) {
    g_state = state;
}

static void handle_rc_override(mavlink_message_t *msg) {
    if (!g_state) return;
    mavlink_rc_channels_override_t rc;
    mavlink_msg_rc_channels_override_decode(msg, &rc);
    if (rc.chan1_raw > 0) g_state->rc_steering = rc.chan1_raw;
    if (rc.chan3_raw > 0) g_state->rc_throttle = rc.chan3_raw;
}

static void handle_set_mode(mavlink_message_t *msg) {
    if (!g_state) return;
    mavlink_set_mode_t mode;
    mavlink_msg_set_mode_decode(msg, &mode);
    g_state->mode = mode.custom_mode;
}

void mavlink_handler_init(void) {
    memset(rx_status, 0, sizeof(rx_status));
}

void mavlink_handle_byte(uint8_t c, int link_id) {
    if (mavlink_parse_char(link_id, c, &rx_msg[link_id], &rx_status[link_id])) {
        mavlink_message_t *msg = &rx_msg[link_id];
        switch (msg->msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT:
                break;
            case MAVLINK_MSG_ID_COMMAND_LONG:
                handle_command_long(msg);
                break;
            case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                handle_param_request_list(msg, link_id);
                break;
            case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
                handle_param_request_read(msg, link_id);
                break;
            case MAVLINK_MSG_ID_PARAM_SET:
                handle_param_set(msg, link_id);
                break;
            case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
                handle_rc_override(msg);
                break;
            case MAVLINK_MSG_ID_SET_MODE:
                handle_set_mode(msg);
                break;
        }
    }
}

void mavlink_send_heartbeat(rover_state_t *state) {
    uint32_t mode_flags = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if (state->armed) mode_flags |= MAV_MODE_FLAG_SAFETY_ARMED;
    if (state->mode == MODE_MANUAL) mode_flags |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID, &msg,
        VEHICLE_TYPE, MAV_AUTOPILOT_ARDUPILOTMEGA,
        mode_flags, state->mode, MAV_STATE_ACTIVE);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mavlink_send_uart_bytes(MAVLINK_COMM_0, buf, len);
    mavlink_send_uart_bytes(MAVLINK_COMM_1, buf, len);
}

void mavlink_send_sys_status(rover_state_t *state) {
    mavlink_message_t msg;
    mavlink_msg_sys_status_pack(SYSTEM_ID, COMPONENT_ID, &msg,
        0, 0, 0, 0,
        (uint16_t)(state->batt_voltage * 1000), -1, state->batt_pct,
        0, 0, 0, 0, 0, 0, 0, 0, 0);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mavlink_send_uart_bytes(MAVLINK_COMM_0, buf, len);
    mavlink_send_uart_bytes(MAVLINK_COMM_1, buf, len);
}

void mavlink_send_rc_channels(rover_state_t *state) {
    mavlink_message_t msg;
    mavlink_msg_rc_channels_pack(SYSTEM_ID, COMPONENT_ID, &msg,
        state->uptime_ms, 2,
        state->rc_steering, 0, state->rc_throttle, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mavlink_send_uart_bytes(MAVLINK_COMM_0, buf, len);
    mavlink_send_uart_bytes(MAVLINK_COMM_1, buf, len);
}

void mavlink_send_statustext(const char *text, uint8_t severity) {
    mavlink_message_t msg;
    mavlink_msg_statustext_pack(SYSTEM_ID, COMPONENT_ID, &msg,
        severity, text, 0, 0);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mavlink_send_uart_bytes(MAVLINK_COMM_0, buf, len);
    mavlink_send_uart_bytes(MAVLINK_COMM_1, buf, len);
}
