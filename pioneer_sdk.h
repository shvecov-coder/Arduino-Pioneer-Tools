#ifndef __PIONEERSDK__
#define __PIONEERSDK__

#include <String.h>
#include <WiFiUdp.h>
#include <MAVLink.h>

WiFiUDP udp;

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

  private:
    int port;
    String ip;
    bool is_connected;
};

#endif /* PIONEERSDK */