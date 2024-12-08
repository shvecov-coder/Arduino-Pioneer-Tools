#include <WiFi.h>
#include "pioneer_sdk.h"

#define WIFI_SSID "PioneerMini8caab591fb70"
#define WIFI_PASS "12345678"

Pioneer mini = Pioneer("192.168.4.1", 8001);

void setup()
{
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connected: ");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println(WiFi.localIP());
}

void loop()
{
  if (Serial.available() > 0)
  {
    int command = Serial.parseInt();
    if (command == 1)
    {
      float battery = mini.get_battery_status();
      Serial.println(battery);
    }
  }
}
