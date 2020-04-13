#include <Arduino.h>
#include <mcp2515.h>


can_frame canMsg;

void setup() {
  Serial.begin(115200);
  
  mcp2515_init();
  mcp2515_reset();
  mcp2515_set_bitrate(CAN_125KBPS);
  mcp2515_set_normal_mode();
  
  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
}

void loop() {
  if (mcp2515_read_message(&canMsg) == ERROR_OK) {
    Serial.print(canMsg.can_id, HEX); // print ID
    Serial.print(" "); 
    Serial.print(canMsg.can_dlc, HEX); // print DLC
    Serial.print(" ");
    
    for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
      Serial.print(canMsg.data[i],HEX);
      Serial.print(" ");
    }

    Serial.println();      
  }
}
