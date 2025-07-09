#include <WiFiUdp.h>
#include <MAVLink.h>

#include "Geoscan_Mini.h"

// Вспомогательная функция-обёртка
extern "C" void wrapper_heartbeat_thread(void* arg)
{
    ((Pioneer*)arg)->heartbeat_thread(); // Вызываем метод класса
}

// Конструктор
Pioneer::Pioneer(const char* ip, uint16_t port)
{
    this->ip = ip;
    this->port = port;
    this->udp = new WiFiUDP();
    this->is_connected = false;
    this->mavlink_timeout = 5000;

    TaskHandle_t xHandle = NULL;
    xTaskCreate(wrapper_heartbeat_thread, "heartbeat_thread", 4096, this, 1, &xHandle); // Используем новую обёртку
    configASSERT(xHandle);
}

// Деструктор
Pioneer::~Pioneer()
{
    delete udp;
}

// Поток отправки heartbeats
void Pioneer::heartbeat_thread()
{
    while(true)
    {
        uint32_t lastSent = 0;
        if(millis() - lastSent <= 1000) continue;
        
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        
        mavlink_msg_heartbeat_pack(1, 1, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        
        udp->beginPacket(ip, port);
        udp->write(buf, len);
        udp->endPacket();

        lastSent = millis();
    }
}

// Отправка команды типа COMMAND_LONG
void Pioneer::send_command_long(uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(0, 0, &msg, 0, 0, command, confirmation, param1, param2, param3, param4, param5, param6, param7);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    udp->beginPacket(ip, port);    // Обращаемся непосредственно к полю объекта
    udp->write(buf, len);
    udp->endPacket();
}

// Включение моторов
void Pioneer::arm()
{
    send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);
}

// Выключение моторов
void Pioneer::disarm()
{
    send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0);
}

// Запрашиваем уровень заряда батареи
float Pioneer::get_battery_status()
{
    mavlink_message_t msg;
    mavlink_command_long_t cmd;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    memset(&cmd, 0, sizeof(cmd));
    cmd.target_system = 0;
    cmd.target_component = 0;
    cmd.command = MAV_CMD_REQUEST_MESSAGE;
    cmd.param1 = 147;

    static uint32_t lastSent = millis();
    while (millis() - lastSent < this->mavlink_timeout)
    {
        mavlink_msg_command_long_encode(1, 1, &msg, &cmd);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

        udp->beginPacket(ip, port);
        udp->write(buf, len);
        udp->endPacket();

        memset(&msg, 0, sizeof(msg));
        memset(&buf, 0, sizeof(buf));

        int packetSize = udp->parsePacket();
        if(!packetSize) continue;

        udp->read(buf, MAVLINK_MAX_PACKET_LEN);
        mavlink_status_t status;

        for(int i = 0; i < packetSize; i++)
        {
            if(mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
            {
                switch(msg.msgid)
                {
                    case MAVLINK_MSG_ID_BATTERY_STATUS:
                        mavlink_battery_status_t battery_status;
                        mavlink_msg_battery_status_decode(&msg, &battery_status);
                        return float(battery_status.voltages[0]) / 100.0f;
                    default:
                        break;
                }
            }
        }
    }
    return NAN;
}


void Pioneer::takeoff()
{
  send_command_long(MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0);
}

void Pioneer::land()
{
  send_command_long(MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0);
}

void Pioneer::led_control(byte led_id, byte r, byte g, byte b){
  send_command_long(MAV_CMD_USER_1, 0, led_id, r, g, b, 0, 0, 0);
}
