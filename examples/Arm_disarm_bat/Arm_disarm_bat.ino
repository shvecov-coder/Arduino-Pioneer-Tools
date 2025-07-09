#include <WiFi.h>
#include <Geoscan_Mini.h>

#define WIFI_SSID ""          // введите название WiFi сети Pioneer Mini
#define WIFI_PASS "12345678"

Pioneer Mini("192.168.4.1", 8001);

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
      Mini.arm();
      Serial.println("arm");
    }
    if (command == 2)
    {
      Mini.disarm();
      Serial.println("disarm");
    }
    if (command == 3)
    {
      Serial.println(Mini.get_battery_status());
    }
  }
}