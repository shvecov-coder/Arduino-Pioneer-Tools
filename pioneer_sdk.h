#ifndef __PIONEERSDK__
#define __PIONEERSDK__

#include <String.h>
#include <WiFiUdp.h>
#include <MAVLink.h>

WiFiUDP udp;
//test
void send_command_long(uint16_t command,
                       uint8_t confirmation,
                       float param1,
                       float param2,
                       float param3,
                       float param4,
                       float param5,
                       float param6,
                       float param7)
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(0, 0, &msg, 0, 0, command, confirmation, param1, param2, param3, param4, param5, param6, param7);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  udp.beginPacket("192.168.4.1", 8001);
  udp.write(buf, len);
  udp.endPacket();
}

class Pioneer
{
  public:
    Pioneer() {}

    static struct args
    {
      String ip;
      int port;
    }args;

    static void heartbeat_thread(void * pvParameters)
    {
      while (true)
      {
        static uint32_t lastSent = 0;
        if (millis() - lastSent < 1000) return;

        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];

        mavlink_msg_heartbeat_pack(1, 1, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	      udp.beginPacket("192.168.4.1", 8001);
	      udp.write(buf, len);
	      udp.endPacket();
  
        lastSent = millis();
      }
    }

    Pioneer(String ip, int port)
    {
      this->port = port;
      this->ip = String(ip);
      this->is_connected = false;
      this->mavlink_timeout = 5000;

      TaskHandle_t xHandle = NULL;
      xTaskCreate(heartbeat_thread, "heartbeat_thread", 4096, NULL, 1, &xHandle);
      configASSERT(xHandle);

      if(xHandle != NULL)
      {
        vTaskDelete(xHandle);
      }
    }
  
    ~Pioneer() {}

    void arm()
    {
      send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);
    }

    void disarm()
    {
      send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    float get_battery_status()
    {
    // Команда для запроса информации о батарее
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
	      udp.beginPacket("192.168.4.1", 8001);
	      udp.write(buf, len);
	      udp.endPacket();

        memset(&msg, 0, sizeof(msg));
        memset(&buf, 0, sizeof(buf));

        int packetSize = udp.parsePacket();
        if (!packetSize) continue;
        udp.read(buf, MAVLINK_MAX_PACKET_LEN);
        mavlink_status_t status;

        for (int i = 0; i < packetSize; i++)
        {
          if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
          {
            switch (msg.msgid)
            {
              case MAVLINK_MSG_ID_BATTERY_STATUS:
                mavlink_battery_status_t battary_status;
                mavlink_msg_battery_status_decode(&msg, &battary_status);
                return float(battary_status.voltages[0] / 100.0);
              default:
                break;
            }
          }
        }
      }   
    }
  private:
    int port;
    String ip;
    bool is_connected;
    int mavlink_timeout;
};

#endif /* PIONEERSDK */