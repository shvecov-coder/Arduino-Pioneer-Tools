#ifndef __GEOSCANMINI__
#define __GEOSCANMINI__

#include <WiFiUdp.h> 
#include <MAVLink.h> 

class Pioneer
{
public:
    Pioneer(const char* ip, uint16_t port);
    ~Pioneer();
    void arm();
    void disarm();
    float get_battery_status();
    void send_command_long(uint16_t command, uint8_t confirmation, float p1, float p2, float p3, float p4, float p5, float p6, float p7);
    void heartbeat_thread();
    void takeoff();
    void led_control(byte led_id, byte r, byte g, byte b);
    void land();
private:
    bool is_connected;
    uint16_t mavlink_timeout;
    WiFiUDP* udp;
    const char* ip;
    uint16_t port;
};

#endif /* __GEOSCANMINI__ */
