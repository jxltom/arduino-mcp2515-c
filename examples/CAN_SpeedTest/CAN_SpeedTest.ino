#include <SPI.h>
#include "mcp2515.h"

struct can_frame canMsg;
int cntr = 0;
unsigned long oldTime = 0;

void setup()
{
  Serial.begin(115200);

  mcp2515_init();
  mcp2515_reset();
  mcp2515_set_bitrate(CAN_125KBPS);
  mcp2515_set_normal_mode();

  Serial.println("------- CAN Speedtest ----------");
}

void loop()
{
  if (mcp2515_read_message(&canMsg) == ERROR_OK)
  {
    cntr++;
  }

  if ((millis() - oldTime) > 1000)
  {
    oldTime = millis();
    Serial.print(cntr);
    Serial.println(" msg/sec");
    cntr = 0;
  }
}
