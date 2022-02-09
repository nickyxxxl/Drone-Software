#include <MPU6050_tockn.h>
#include <Wire.h>
//#include <ESP32Servo.h>
#include <sbus.h>

///////////////////////////User defined/////////////////////////////////////

//Define the pins for each motor
<<<<<<< HEAD
#define _motor1 26
#define _motor2 25
#define _motor3 32
#define _motor4 33
=======
#define _motor1 25
>>>>>>> 2b0607e31ca989fe222d0c9c314e2a54dd60536e

//Define pin for sbus rx and tx channel (we only care about rx)
const int8_t rxpin {16};
const int8_t txpin {17}; //We don't use this, set it to whatever.
HardwareSerial mySerial(1);

<<<<<<< HEAD
//Create Servo object, used to send pwm signal
/*
Servo motor1;                       //  4 ----- 2
Servo motor2;                       //   |  ^  |
Servo motor3;                       //   |     |
Servo motor4;                       //  3 ----- 1
*/
//Target values user input
float throttle;
=======
const int minValue {1000};     //min max values ESC
const int maxValue {2000};

const float axisSensitivity {10000}; //How much degrees/s should max stick be?

//Create Servo object, used to send pwm signal
Servo motor1;
>>>>>>> 2b0607e31ca989fe222d0c9c314e2a54dd60536e

float m1;
float throttle;

bool failsafe = false;

/* SbusRx object on UART 1 */
bfs::SbusRx sbus_rx(&mySerial);
std::array<int16_t, bfs::SbusRx::NUM_CH()> sbus_data;


///////////////////////////SETUP/////////////////////////////////////

void initializeServos() {
  //Enable motors
  /*
  motor1.attach(_motor1);
<<<<<<< HEAD
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
=======

  motor1.writeMicroseconds(1000);      //Make sure motors are not spinning

>>>>>>> 2b0607e31ca989fe222d0c9c314e2a54dd60536e
}

//Store Sbus data in array sbus_rx
void getSbus() {
  if (sbus_rx.Read()) {
    if (sbus_rx.lost_frame())return;
    sbus_data = sbus_rx.ch();
  }
<<<<<<< HEAD
  if (sbus_rx.lost_frame()) {
    return;
  }
}

=======
  if (sbus_rx.failsafe()) {
    failsafe = true;
    motor1.writeMicroseconds(0);      //Disable motors if receivers looses connection

  }
  else failsafe = false;

}

////////////////////////////Data processing////////////////////////////

//////Channel layout//////range/////////desired/////
//channel 1: Roll        1000-2000    -100 tot 100
//channel 2: Pitch       1000-2000    -100 tot 100
//channel 3: throttle    1000-2000    1000 tot 1750
//channel 4: Yaw         1000-2000    -100 tot 100
//channel 5: arm         1000-2000    <1400 of >1600
//channel 6: mode        1000-2000
//channel 7: failsafe    1000-2000

void mapInput() {

  throttle = map(sbus_data[2], 180, 1820, 780, 2000);
}

///////////////////////////////////////////////////////////////////////


>>>>>>> 2b0607e31ca989fe222d0c9c314e2a54dd60536e
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
<<<<<<< HEAD
  
  Serial.begin(115200);
  
  Serial.println("Initializing servos");
  //initializeServos();             //Start connection to motors
  
  Serial.println("Starting sbus connection.");
  sbus_rx.Begin(rxpin, txpin);    //Begin Sbus communication
=======
  int now = millis();

  Serial.begin(115200);
  Serial.println("tekst in" + String(now));

  Serial.println("Starting sbus connection.");
  sbus_rx.Begin(rxpin, txpin);    //Begin Sbus communication

  Serial.println("Initializing servos");
  initializeServos();             //Start connection to motors
  Serial.println("Finished initializing servos");

  Serial.println("Finished setup in: " + String(millis() - now) + " ms");

>>>>>>> 2b0607e31ca989fe222d0c9c314e2a54dd60536e
}

void loop() {

  //////Get system data//////
  getSbus();
<<<<<<< HEAD
  throttle = map(sbus_data[2], 180, 1820, 1, 255);
    
=======
  mapInput();

  if (failsafe)return;  //Disable everything if signal is lost or arm button is off.

>>>>>>> 2b0607e31ca989fe222d0c9c314e2a54dd60536e
  //////Apply to motors//////
  applyMotors();
  delay(10);
}
