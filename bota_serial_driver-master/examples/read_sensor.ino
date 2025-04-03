#include "BotaForceTorqueSensorComm.h"

class myBotaForceTorqueSensorComm : public BotaForceTorqueSensorComm
{
  public:
  int serialReadBytes(uint8_t* data, size_t len) override {return Serial.readBytes(data, len);}
  int serialAvailable() override {return Serial.available();}
} sensor;

void setup() {
  Serial.begin(230400);
  while(!Serial);
}

void loop() {
  // put your main code here, to run repeatedly:
  switch(sensor.readFrame())
  {
    case BotaForceTorqueSensorComm::VALID_FRAME:
      if (sensor.frame.data.status.val>0)
      {
        Serial.println("No valid forces: ");
        Serial.print("app_took_too_long: ");
        Serial.println(sensor.frame.data.status.app_took_too_long);
        Serial.print("overrange: ");
        Serial.println(sensor.frame.data.status.overrange);
        Serial.print("invalid_measurements: ");
        Serial.println(sensor.frame.data.status.invalid_measurements);
        Serial.print("raw_measurements: ");
        Serial.println(sensor.frame.data.status.raw_measurements);
      }
      else
      {
        for (uint8_t i=0; i<6; i++)
        {
          Serial.print(sensor.frame.data.forces[i]);
          Serial.print("\t");
        }
        Serial.println();
      }
      break;
    case BotaForceTorqueSensorComm::NOT_VALID_FRAME:
      Serial.print("No valid frame: ");
      Serial.println(sensor.get_crc_count());
      break;
    case BotaForceTorqueSensorComm::NOT_ALLIGNED_FRAME:
      Serial.println("lost sync, trying to reconnect");
      break;
    case BotaForceTorqueSensorComm::NO_FRAME:
      break;
  }
}
