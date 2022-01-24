#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

void setup(){

  Serial.begin(115200);

  Serial.println("Initializing current sensor.");
  while(!ina219.begin());
  Serial.println("Finished initializing current sensor");
}

void loop(){

  Serial.println("Voltage= " + String(ina219.getBusVoltage_V();
  delay(1000);
  
}
