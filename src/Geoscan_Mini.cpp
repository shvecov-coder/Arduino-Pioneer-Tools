#include "Geoscan_Mini_Test_V2.h"

// <-- НОВОЕ: Массив состояний автопилота для get_autopilot_state()
static const char* AUTOPILOT_STATE_NAMES[] = {
    "ROOT", "DISARMED", "IDLE", "TEST_ACTUATION", "TEST_PARACHUTE",
    "TEST_ENGINE", "PARACHUTE", "WAIT_FOR_LANDING", "LANDED", "CATAPULT",
    "PREFLIGHT", "ARMED", "TAKEOFF", "WAIT_FOR_GPS", "WIND_MEASURE",
    "MISSION", "ASCEND", "DESCEND", "RTL", "UNCONDITIONAL_RTL",
    "MANUAL_HEADING", "MANUAL_ROLL", "MANUAL_SPEED", "LANDING", "ON_DEMAND"
};

Geoscan_Mini_Test_V2::Geoscan_Mini_Test_V2() {
    // Инициализация состояний
    memset((void*)&_preflight_state, 0, sizeof(PreflightState));
}

Geoscan_Mini_Test_V2::~Geoscan_Mini_Test_V2() {
    close_connection();
}

void Geoscan_Mini_Test_V2::begin(const char* ip, uint16_t port) {
    drone_ip.fromString(ip);
    drone_port = port;
    udp.begin(drone_port);

    xTaskCreate(message_handler_task, "MAVLink Handler", 4096, this, 1, &message_handler_task_handle);
}

void Geoscan_Mini_Test_V2::close_connection() {
    if (message_handler_task_handle != NULL) {
        vTaskDelete(message_handler_task_handle);
        message_handler_task_handle = NULL;
    }
    udp.stop();
    is_connected_flag = false;
    mav_log("connection", "Connection closed");
}

bool Geoscan_Mini_Test_V2::connected() {
    return is_connected_flag;
}

// <-- НОВОЕ: Реализация всех недостающих методов -->

void Geoscan_Mini_Test_V2::set_logger(bool value) {
    _logger = value;
}

void Geoscan_Mini_Test_V2::set_log_connection(bool value) {
    _log_connection = value;
}

void Geoscan_Mini_Test_V2::mav_log(const char* msg_type, const char* msg) {
    if (strcmp(msg_type, "connection") == 0 && !_log_connection) return;
    if (strcmp(msg_type, "connection") != 0 && !_logger) return;
    
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "[Geoscan] <%s> %s", msg_type, msg);
    Serial.println(buffer);
}

void Geoscan_Mini_Test_V2::message_handler_task(void* pvParameters) {
    ((Geoscan_Mini_Test_V2*)pvParameters)->message_handler_loop();
}

void Geoscan_Mini_Test_V2::message_handler_loop() {
    for (;;) {
        if (millis() - last_heartbeat_send_time > HEARTBEAT_INTERVAL) {
            send_heartbeat();
        }

        if (is_connected_flag && (millis() - last_msg_time > CONNECTION_TIMEOUT)) {
            is_connected_flag = false;
            mav_log("connection", "DISCONNECTED");
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
        mav_log("connection", "CONNECTED");
    }

    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            _autopilot_state_enum = hb.custom_mode & 0xFF; // Младший байт
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_ACK: {
            mavlink_command_ack_t ack;
            mavlink_msg_command_ack_decode(&msg, &ack);
            last_ack_command = ack.command;
            last_ack_result = ack.result;

            if (ack.command == MAV_CMD_PREFLIGHT_STORAGE) { // Command 400
                uint32_t p2 = ack.result_param2;
                _preflight_state.BatteryLow = p2 & 0b00000001;
                _preflight_state.NavSystem = p2 & 0b00000010;
                _preflight_state.Area = p2 & 0b00000100;
                _preflight_state.Attitude = p2 & 0b00001000;
                _preflight_state.RcExpected = p2 & 0b00010000;
                _preflight_state.RcMode = p2 & 0b00100000;
                _preflight_state.RcUnexpected = p2 & 0b01000000;
                _preflight_state.UavStartAllowed = p2 & 0b10000000;
            }
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
        // <-- НОВОЕ: Обработка сообщений для геттеров -->
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
            mavlink_local_position_ned_t pos;
            mavlink_msg_local_position_ned_decode(&msg, &pos);
            _local_x = pos.y; // Конвертация NED -> ENU
            _local_y = pos.x;
            _local_z = -pos.z;
            _new_position_data = true;
            break;
        }
        case MAVLINK_MSG_ID_DISTANCE_SENSOR: {
            mavlink_distance_sensor_t dist;
            mavlink_msg_distance_sensor_decode(&msg, &dist);
            _distance_sensor_m = dist.current_distance / 100.0f;
            _new_distance_data = true;
            break;
        }
        case MAVLINK_MSG_ID_BATTERY_STATUS: {
            mavlink_battery_status_t bat;
            mavlink_msg_battery_status_decode(&msg, &bat);
            if (bat.voltages[0] != UINT16_MAX) {
                _battery_voltage = bat.voltages[0] / 100.0f;
                 _new_battery_data = true;
            }
            break;
        }
        case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD: {
            mavlink_optical_flow_rad_t opt;
            mavlink_msg_optical_flow_rad_decode(&msg, &opt);
            _optical_flow_data.integration_time_us = opt.integration_time_us;
            _optical_flow_data.integrated_x = opt.integrated_x;
            _optical_flow_data.integrated_y = opt.integrated_y;
            _optical_flow_data.integrated_xgyro = opt.integrated_xgyro;
            _optical_flow_data.integrated_ygyro = opt.integrated_ygyro;
            _optical_flow_data.integrated_zgyro = opt.integrated_zgyro;
            _optical_flow_data.temperature = opt.temperature;
            _optical_flow_data.quality = opt.quality;
            _optical_flow_data.distance = opt.distance;
            _new_optical_data = true;
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
    mavlink_msg_set_position_target_local_ned_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0, 1, MAV_COMP_ID_AUTOPILOT1, coordinate_system, mask, x, y, z, vx, vy, vz, 0, 0, 0, yaw, yaw_rate);
    send_message(msg);
    return true;
}

bool Geoscan_Mini_Test_V2::arm() { return send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 1.0f); }
bool Geoscan_Mini_Test_V2::disarm() { return send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 0.0f); }
bool Geoscan_Mini_Test_V2::takeoff() { return send_command_long(MAV_CMD_NAV_TAKEOFF); }
bool Geoscan_Mini_Test_V2::land() { return send_command_long(MAV_CMD_NAV_LAND); }
bool Geoscan_Mini_Test_V2::reboot_board() { return send_command_long(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1.0f, 0, 0, 0, 0, 0, 0, 1); }
bool Geoscan_Mini_Test_V2::led_control(uint8_t led_id, uint8_t r, uint8_t g, uint8_t b) { return send_command_long(MAV_CMD_USER_1, (float)led_id, (float)r, (float)g, (float)b); }
bool Geoscan_Mini_Test_V2::lua_script_control(bool start) { return send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, start ? 1.0f : 0.0f, 0, 0, 0, 0, 0, 0, 25); }

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
    uint16_t mask = 0b0000010111000111;
    return send_position_target_local_ned(MAV_FRAME_LOCAL_NED, mask, 0, 0, 0, vy, vx, -vz, 0, yaw_rate);
}

bool Geoscan_Mini_Test_V2::set_manual_speed_body_fixed(float vx, float vy, float vz, float yaw_rate) {
    uint16_t mask = 0b0000010111000111;
    return send_position_target_local_ned(MAV_FRAME_BODY_FRD, mask, 0, 0, 0, vy, vx, -vz, 0, yaw_rate);
}

// --- НОВЫЕ МЕТОДЫ ---
void Geoscan_Mini_Test_V2::send_rc_channels(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4, uint16_t ch5, uint16_t ch6, uint16_t ch7, uint16_t ch8) {
    mavlink_message_t msg;
    // <-- ИСПРАВЛЕНИЕ: Добавлены 10 каналов со значением UINT16_MAX
    mavlink_msg_rc_channels_override_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 1, MAV_COMP_ID_AUTOPILOT1, 
                                          ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8,
                                          UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,
                                          UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,
                                          UINT16_MAX, UINT16_MAX);
    send_message(msg);
}

bool Geoscan_Mini_Test_V2::raspberry_poweroff() { return send_command_long(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 0, 0, 0, 0, 0, 0, 42); }
bool Geoscan_Mini_Test_V2::raspberry_reboot() { return send_command_long(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 0, 0, 0, 0, 0, 0, 43); }
bool Geoscan_Mini_Test_V2::raspberry_start_capture(float interval, uint16_t total_images, uint16_t sequence_number) { return send_command_long(MAV_CMD_IMAGE_START_CAPTURE, 0, interval, (float)total_images, (float)sequence_number); }
bool Geoscan_Mini_Test_V2::raspberry_stop_capture() { return send_command_long(MAV_CMD_IMAGE_STOP_CAPTURE); }
bool Geoscan_Mini_Test_V2::raspberry_led_custom(uint8_t mode, uint16_t timer, uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2) {
    float p2 = (((r1 << 8) | g1) << 8) | b1;
    float p3 = (((r2 << 8) | g2) << 8) | b2;
    return send_command_long(MAV_CMD_USER_3, 0, p2, p3, 0, (float)mode, (float)timer, 0, 0);
}

bool Geoscan_Mini_Test_V2::get_local_position_lps(float& x, float& y, float& z, bool get_last_received) {
    if (!_new_position_data && !get_last_received) return false;
    taskENTER_CRITICAL(&mux);
    x = _local_x;
    y = _local_y;
    z = _local_z;
    if (!get_last_received) _new_position_data = false;
    taskEXIT_CRITICAL(&mux);
    return true;
}

bool Geoscan_Mini_Test_V2::get_dist_sensor_data(float& distance, bool get_last_received) {
    if (!_new_distance_data && !get_last_received) return false;
    taskENTER_CRITICAL(&mux);
    distance = _distance_sensor_m;
    if (!get_last_received) _new_distance_data = false;
    taskEXIT_CRITICAL(&mux);
    return true;
}

bool Geoscan_Mini_Test_V2::get_battery_status(float& voltage, bool get_last_received) {
    if (!_new_battery_data && !get_last_received) return false;
    taskENTER_CRITICAL(&mux);
    voltage = _battery_voltage;
    if (!get_last_received) _new_battery_data = false;
    taskEXIT_CRITICAL(&mux);
    return true;
}

bool Geoscan_Mini_Test_V2::get_optical_data(OpticalFlowData& data, bool get_last_received) {
    if (!_new_optical_data && !get_last_received) return false;
    // <-- ИСПРАВЛЕНИЕ: Безопасное копирование внутри критической секции
    taskENTER_CRITICAL(&mux);
    data.integration_time_us = _optical_flow_data.integration_time_us;
    data.integrated_x = _optical_flow_data.integrated_x;
    data.integrated_y = _optical_flow_data.integrated_y;
    data.integrated_xgyro = _optical_flow_data.integrated_xgyro;
    data.integrated_ygyro = _optical_flow_data.integrated_ygyro;
    data.integrated_zgyro = _optical_flow_data.integrated_zgyro;
    data.temperature = _optical_flow_data.temperature;
    data.quality = _optical_flow_data.quality;
    data.distance = _optical_flow_data.distance;
    if (!get_last_received) _new_optical_data = false;
    taskEXIT_CRITICAL(&mux);
    return true;
}

PreflightState Geoscan_Mini_Test_V2::get_preflight_state() {
    send_command_long(MAV_CMD_PREFLIGHT_STORAGE); // Запрашиваем обновление
    PreflightState temp_state;
    // <-- ИСПРАВЛЕНИЕ: Безопасное копирование внутри критической секции
    taskENTER_CRITICAL(&mux);
    temp_state.BatteryLow = _preflight_state.BatteryLow;
    temp_state.NavSystem = _preflight_state.NavSystem;
    temp_state.Area = _preflight_state.Area;
    temp_state.Attitude = _preflight_state.Attitude;
    temp_state.RcExpected = _preflight_state.RcExpected;
    temp_state.RcMode = _preflight_state.RcMode;
    temp_state.RcUnexpected = _preflight_state.RcUnexpected;
    temp_state.UavStartAllowed = _preflight_state.UavStartAllowed;
    taskEXIT_CRITICAL(&mux);
    return temp_state;
}

const char* Geoscan_Mini_Test_V2::get_autopilot_state() {
    uint8_t state_copy;
    taskENTER_CRITICAL(&mux);
    state_copy = _autopilot_state_enum;
    taskEXIT_CRITICAL(&mux);
    if (state_copy < sizeof(AUTOPILOT_STATE_NAMES) / sizeof(AUTOPILOT_STATE_NAMES[0])) {
        return AUTOPILOT_STATE_NAMES[state_copy];
    }
    return "UNKNOWN";
}
