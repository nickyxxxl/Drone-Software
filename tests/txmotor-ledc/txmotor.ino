#include <MPU6050_tockn.h>
#include <Wire.h>
#include <sbus.h>

///////////////////////////User defined/////////////////////////////////////

//Define the pins for each motor
#define _motor1 26
#define _motor2 25
#define _motor3 32
#define _motor4 33

//Define pin for sbus rx and tx channel (we only care about rx)
const int8_t rxpin {16};
const int8_t txpin {17}; //We don't use this, set it to whatever.
HardwareSerial mySerial(1);

float m1;
float throttle;

bool failsafe = false;

/* SbusRx object on UART 1 */
bfs::SbusRx sbus_rx(&mySerial);
std::array<int16_t, bfs::SbusRx::NUM_CH()> sbus_data;

void initializeServos() {
  delay(5000);
  ledcSetup(1,1000,10); //(channel, frequency, resolution)
  ledcSetup(2,1000,10);
  ledcSetup(3,1000,10);
  ledcSetup(4,1000,10);
  ledcAttachPin(_motor1, 1); //(pin, channel)
  ledcAttachPin(_motor2, 2);
  ledcAttachPin(_motor3, 3);
  ledcAttachPin(_motor4, 4);
}

//Store Sbus data in array sbus_rx
void getSbus() {
  if (sbus_rx.Read()) {
    if (sbus_rx.lost_frame())return;
    sbus_data = sbus_rx.ch();
  }
}
void applyMotors() {
  if (sbus_rx.lost_frame()) {
    return;
  }
  m1 = throttle;
  Serial.println(m1);
  
  ledcWrite(1,m1);
  ledcWrite(2,m1);
  ledcWrite(3,m1);
  ledcWrite(4,m1);
}

void setup() {
  
  Serial.begin(115200);
  
  Serial.println("Initializing servos");
  initializeServos();             //Start connection to motors
  
  Serial.println("Starting sbus connection.");
  sbus_rx.Begin(rxpin, txpin);    //Begin Sbus communication
}

void loop() {

  //////Get system data//////
  getSbus();
  throttle = map(sbus_data[2], 180, 1820, 0, 1023);
    
  //////Apply to motors//////
  applyMotors();
  delay(10);
}
