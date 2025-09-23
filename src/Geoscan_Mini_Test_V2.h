#ifndef GEOSCAN_MINI_TEST_V2_H
#define GEOSCAN_MINI_TEST_V2_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define MAVLINK_COMM_NUM_BUFFERS 1
#include <MAVLink.h>

struct OpticalFlowData {
    uint32_t integration_time_us;
    float integrated_x;
    float integrated_y;
    float integrated_xgyro;
    float integrated_ygyro;
    float integrated_zgyro;
    int16_t temperature;
    uint8_t quality;
    float distance;
};

struct PreflightState {
    bool BatteryLow;
    bool NavSystem;
    bool Area;
    bool Attitude;
    bool RcExpected;
    bool RcMode;
    bool RcUnexpected;
    bool UavStartAllowed;
};


class Geoscan_Mini_Test_V2 {
public:
    Geoscan_Mini_Test_V2();
    ~Geoscan_Mini_Test_V2();

    void begin(const char* ip = "192.168.4.1", uint16_t port = 8001);
    void close_connection();
    bool connected();
    void set_logger(bool value);
    void set_log_connection(bool value);

    bool arm();
    bool disarm();
    bool takeoff();
    bool land();

    bool go_to_local_point(float x, float y, float z, float yaw);
    bool go_to_local_point_body_fixed(float x, float y, float z, float yaw);
    bool set_manual_speed(float vx, float vy, float vz, float yaw_rate);
    bool set_manual_speed_body_fixed(float vx, float vy, float vz, float yaw_rate);
    bool point_reached();

    bool led_control(uint8_t led_id = 255, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0);
    bool lua_script_control(bool start);

    bool reboot_board();
    void send_rc_channels(uint16_t ch1 = UINT16_MAX, uint16_t ch2 = UINT16_MAX, uint16_t ch3 = UINT16_MAX, uint16_t ch4 = UINT16_MAX, uint16_t ch5 = UINT16_MAX, uint16_t ch6 = UINT16_MAX, uint16_t ch7 = UINT16_MAX, uint16_t ch8 = UINT16_MAX);

    bool raspberry_poweroff();
    bool raspberry_reboot();
    bool raspberry_start_capture(float interval = 0.1, uint16_t total_images = 0, uint16_t sequence_number = 0);
    bool raspberry_stop_capture();
    bool raspberry_led_custom(uint8_t mode, uint16_t timer, uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2);

    bool get_local_position_lps(float& x, float& y, float& z, bool get_last_received = false);
    bool get_dist_sensor_data(float& distance, bool get_last_received = false);
    bool get_battery_status(float& voltage, bool get_last_received = false);
    bool get_optical_data(OpticalFlowData& data, bool get_last_received = false);
    PreflightState get_preflight_state();
    const char* get_autopilot_state();

private:
    WiFiUDP udp;
    IPAddress drone_ip;
    uint16_t drone_port;
    bool is_connected_flag = false;
    bool _logger = true;
    bool _log_connection = true;

    const uint8_t MAV_SYSTEM_ID = 255;
    const uint8_t MAV_COMPONENT_ID = MAV_COMP_ID_MISSIONPLANNER;
    const uint16_t MAV_CMD_ACK_TIMEOUT = 500;
    const uint8_t MAV_SEND_RETRIES = 10;
    const uint16_t HEARTBEAT_INTERVAL = 1000;
    const uint16_t CONNECTION_TIMEOUT = 3000;

    volatile unsigned long last_msg_time = 0;
    unsigned long last_heartbeat_send_time = 0;
    
    volatile uint16_t last_ack_command = 0;
    volatile int8_t last_ack_result = -1;
    
    volatile bool point_reached_flag = false;
    uint16_t point_seq = 0;

    volatile float _local_x, _local_y, _local_z;
    volatile float _distance_sensor_m;
    volatile float _battery_voltage;
    volatile OpticalFlowData _optical_flow_data;
    volatile PreflightState _preflight_state;
    volatile uint8_t _autopilot_state_enum = 0;

    volatile bool _new_position_data = false;
    volatile bool _new_distance_data = false;
    volatile bool _new_battery_data = false;
    volatile bool _new_optical_data = false;
    
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; // <-- ИЗМЕНЕНИЕ: Добавлен мьютекс

    TaskHandle_t message_handler_task_handle = NULL;
    static void message_handler_task(void* pvParameters);
    void message_handler_loop();
    void handle_mavlink_message(const mavlink_message_t& msg);

    void mav_log(const char* msg_type, const char* msg);
    void send_heartbeat();
    void send_message(const mavlink_message_t& msg);
    bool send_command_long(uint16_t command, float param1 = 0, float param2 = 0, float param3 = 0, float param4 = 0, float param5 = 0, float param6 = 0, float param7 = 0, uint8_t target_component = MAV_COMP_ID_AUTOPILOT1);
    bool send_position_target_local_ned(uint8_t coordinate_system, uint16_t mask, float x = 0, float y = 0, float z = 0, float vx = 0, float vy = 0, float vz = 0, float yaw = 0, float yaw_rate = 0);
};

#endif // GEOSCAN_MINI_TEST_V2_H
