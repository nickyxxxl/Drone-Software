#include <MPU6050_tockn.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <sbus.h>

//Define pin for sbus rx and tx channel (we only care about rx)
const int8_t rxpin {16};
const int8_t txpin {17}; //We don't use this, set it to whatever.
HardwareSerial mySerial(1);

const int minValue {1000};     //min max values ESC
const int maxValue {2000};

//Operational variables
float deltaT {};

float currentTime;
float lastTime;


/* SbusRx object on Serial1 */
bfs::SbusRx sbus_rx(&mySerial);
/* Array for storing SBUS data */
std::array<int16_t, bfs::SbusRx::NUM_CH()> sbus_data;

////////////////////////////SetData///////////////////////////////////

//Store Sbus data in array sbus_rx
void getSbus() {
  if (sbus_rx.Read()) {
    sbus_data = sbus_rx.ch();
  }

}

//function to map a float from range (in_min, in_out) to (out_min, out_max)
float mapFloat(float input, float in_min, float in_max, float out_min, float out_max) {
  return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  delay(5000);

  int now = millis();
  Serial.begin(115200);
  Serial.print("Test2!\n");
  while (!Serial) {}
  Serial.print("Test1!\n");

 // mySerial.begin(100000, SERIAL_8E2, rxpin, txpin, true);

  Serial.print("Starting sbus connection.\n");
  sbus_rx.Begin(rxpin, txpin);    //Begin Sbus communication

  Serial.print("Finished setup in: ");
  Serial.print(millis() - now);
  Serial.print(" ms");
}

void loop() {
  if (sbus_rx.Read()) {
    /* Grab the received data */
    sbus_data = sbus_rx.ch();
    /* Display the received data */
    for (int8_t i = 0; i < bfs::SbusRx::NUM_CH(); i++) {
      Serial.print(sbus_data[i]);
      Serial.print("\t");
    }
    /* Display lost frames and failsafe data */
    Serial.print(sbus_rx.lost_frame());
    Serial.print("\t");
    Serial.println(sbus_rx.failsafe());
    delay(10);
  }
}
