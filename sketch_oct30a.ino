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
    int x = Serial.parseInt();
    if (x == 1)
    {
      Serial.println("ARM()...");
      mini.arm();
    }
    else if (x == 2)
    {
      Serial.println("DISARM()...");
      mini.disarm();
    }
    else
    {
      Serial.println("Error command...");
    }
  }
}
