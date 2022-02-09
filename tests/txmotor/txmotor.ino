#include <MPU6050_tockn.h>
#include <Wire.h>
//#include <ESP32Servo.h>
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

//Create Servo object, used to send pwm signal
/*
Servo motor1;                       //  4 ----- 2
Servo motor2;                       //   |  ^  |
Servo motor3;                       //   |     |
Servo motor4;                       //  3 ----- 1
*/
//Target values user input
float throttle;

float m1;
float m2;
float m3;
float m4;


/* SbusRx object on UART 1 */
bfs::SbusRx sbus_rx(&mySerial);
std::array<int16_t, bfs::SbusRx::NUM_CH()> sbus_data;


///////////////////////////SETUP/////////////////////////////////////

void initializeServos() {
  //Enable motors
  /*
  motor1.attach(_motor1);
  motor2.attach(_motor2);
  motor3.attach(_motor3);
  motor4.attach(_motor4);
  /*
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  */
  delay(10000);
}

//Store Sbus data in array sbus_rx
void getSbus() {
  if (sbus_rx.Read()) {
    sbus_data = sbus_rx.ch();
  }
  if (sbus_rx.lost_frame()) {
    return;
  }
}

void applyMotors() {
  if (sbus_rx.lost_frame()) {
    return;
  }
  m1 = throttle;
  Serial.println(m1);
  //motor1.writeMicroseconds(m1);
  //motor2.writeMicroseconds(m1);
  //motor3.writeMicroseconds(m1);
  //motor4.writeMicroseconds(m1);
  analogWrite(_motor1,m1);
  analogWrite(_motor2,m1);
  analogWrite(_motor3,m1);
  analogWrite(_motor4,m1);
}

void setup() {
  
  Serial.begin(115200);
  
  Serial.println("Initializing servos");
  //initializeServos();             //Start connection to motors
  
  Serial.println("Starting sbus connection.");
  sbus_rx.Begin(rxpin, txpin);    //Begin Sbus communication
}

void loop() {

  //////Get system data//////
  getSbus();
  throttle = map(sbus_data[2], 180, 1820, 1, 255);
    
  //////Apply to motors//////
  applyMotors();
  delay(10);
}
