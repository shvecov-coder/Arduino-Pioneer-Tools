#include "Geoscan_Mini_Test_V2.h"

Geoscan_Mini_Test_V2::Geoscan_Mini_Test_V2() {}

Geoscan_Mini_Test_V2::~Geoscan_Mini_Test_V2() {
    close_connection();
}

void Geoscan_Mini_Test_V2::begin(const char* ip, uint16_t port) {
    drone_ip.fromString(ip);
    drone_port = port;

    udp.begin(drone_port);

    xTaskCreate(
        message_handler_task,
        "MAVLink Handler",
        4096,
        this,
        1,
        &message_handler_task_handle
    );
}

void Geoscan_Mini_Test_V2::close_connection() {
    if (message_handler_task_handle != NULL) {
        vTaskDelete(message_handler_task_handle);
        message_handler_task_handle = NULL;
    }
    udp.stop();
    is_connected_flag = false;
}

bool Geoscan_Mini_Test_V2::connected() {
    return is_connected_flag;
}

void Geoscan_Mini_Test_V2::message_handler_task(void* pvParameters) {
    Geoscan_Mini_Test_V2* instance = (Geoscan_Mini_Test_V2*)pvParameters;
    instance->message_handler_loop();
}

void Geoscan_Mini_Test_V2::message_handler_loop() {
    for (;;) {
        if (millis() - last_heartbeat_send_time > HEARTBEAT_INTERVAL) {
            send_heartbeat();
        }

        if (is_connected_flag && (millis() - last_msg_time > CONNECTION_TIMEOUT)) {
            is_connected_flag = false;
            Serial.println("[Geoscan] <connection> DISCONNECTED");
        }

        if (udp.parsePacket()) {
            mavlink_message_t msg;
            mavlink_status_t status;
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            int len = udp.read(buffer, sizeof(buffer));
            for (int i = 0; i < len; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
                    handle_mavlink_message(msg);
                }
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void Geoscan_Mini_Test_V2::handle_mavlink_message(const mavlink_message_t& msg) {
    last_msg_time = millis();
    if (!is_connected_flag) {
        is_connected_flag = true;
        Serial.println("[Geoscan] <connection> CONNECTED");
    }

    switch (msg.msgid) {
        case MAVLINK_MSG_ID_COMMAND_ACK: {
            mavlink_command_ack_t ack;
            mavlink_msg_command_ack_decode(&msg, &ack);
            last_ack_command = ack.command;
            last_ack_result = ack.result;
            break;
        }
        case MAVLINK_MSG_ID_MISSION_ITEM_REACHED: {
            mavlink_mission_item_reached_t reached;
            mavlink_msg_mission_item_reached_decode(&msg, &reached);
            if (point_seq == 0 || reached.seq > point_seq) {
                point_reached_flag = true;
            }
            point_seq = reached.seq;
            break;
        }
    }
}

void Geoscan_Mini_Test_V2::send_message(const mavlink_message_t& msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    udp.beginPacket(drone_ip, drone_port);
    udp.write(buffer, len);
    udp.endPacket();
}

void Geoscan_Mini_Test_V2::send_heartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0);
    send_message(msg);
    last_heartbeat_send_time = millis();
}

bool Geoscan_Mini_Test_V2::send_command_long(uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7, uint8_t target_component) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 1, target_component, command, 0, param1, param2, param3, param4, param5, param6, param7);

    for (int i = 0; i < MAV_SEND_RETRIES; ++i) {
        last_ack_command = 0;
        last_ack_result = -1;
        send_message(msg);

        unsigned long start_wait = millis();
        while (millis() - start_wait < MAV_CMD_ACK_TIMEOUT) {
            if (last_ack_command == command) {
                if (last_ack_result == MAV_RESULT_IN_PROGRESS) {
                    start_wait = millis();
                    last_ack_command = 0;
                } else {
                    return last_ack_result == MAV_RESULT_ACCEPTED || last_ack_result == MAV_RESULT_DENIED;
                }
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
    return false;
}

bool Geoscan_Mini_Test_V2::send_position_target_local_ned(uint8_t coordinate_system, uint16_t mask, float x, float y, float z, float vx, float vy, float vz, float yaw, float yaw_rate) {
    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack(
        MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg,
        0,
        1, MAV_COMP_ID_AUTOPILOT1,
        coordinate_system,
        mask,
        x, y, z,
        vx, vy, vz,
        0, 0, 0,
        yaw, yaw_rate
    );
    send_message(msg);
    return true;
}

bool Geoscan_Mini_Test_V2::arm() {
    return send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

bool Geoscan_Mini_Test_V2::disarm() {
    return send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 0.0f);
}

bool Geoscan_Mini_Test_V2::takeoff() {
    return send_command_long(MAV_CMD_NAV_TAKEOFF);
}

bool Geoscan_Mini_Test_V2::land() {
    return send_command_long(MAV_CMD_NAV_LAND);
}

bool Geoscan_Mini_Test_V2::led_control(uint8_t led_id, uint8_t r, uint8_t g, uint8_t b) {
    return send_command_long(MAV_CMD_USER_1, (float)led_id, (float)r, (float)g, (float)b);
}

bool Geoscan_Mini_Test_V2::point_reached() {
    if (point_reached_flag) {
        point_reached_flag = false;
        return true;
    }
    return false;
}

bool Geoscan_Mini_Test_V2::go_to_local_point(float x, float y, float z, float yaw) {
    point_reached_flag = false;
    uint16_t mask = 0b0000100111111000;
    return send_position_target_local_ned(MAV_FRAME_LOCAL_NED, mask, y, x, -z, 0, 0, 0, yaw);
}

bool Geoscan_Mini_Test_V2::go_to_local_point_body_fixed(float x, float y, float z, float yaw) {
    point_reached_flag = false;
    uint16_t mask = 0b0000100111111000;
    return send_position_target_local_ned(MAV_FRAME_BODY_FRD, mask, y, x, -z, 0, 0, 0, yaw);
}

bool Geoscan_Mini_Test_V2::set_manual_speed(float vx, float vy, float vz, float yaw_rate) {
    uint16_t mask = 0b0000011111000111;
    return send_position_target_local_ned(MAV_FRAME_LOCAL_NED, mask, 0, 0, 0, vy, vx, -vz, 0, yaw_rate);
}

bool Geoscan_Mini_Test_V2::set_manual_speed_body_fixed(float vx, float vy, float vz, float yaw_rate) {
    uint16_t mask = 0b0000011111000111;
    return send_position_target_local_ned(MAV_FRAME_BODY_FRD, mask, 0, 0, 0, vy, vx, -vz, 0, yaw_rate);
}
