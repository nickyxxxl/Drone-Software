#include <MPU6050_tockn.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <sbus.h>

///////////////////////////User defined/////////////////////////////////////

//Define the pins for each motor
#define _motor1 25

//Define pin for sbus rx and tx channel (we only care about rx)
const int8_t rxpin {16};
const int8_t txpin {17}; //We don't use this, set it to whatever.
HardwareSerial mySerial(1);

const int minValue {1000};     //min max values ESC
const int maxValue {2000};

const float axisSensitivity {1000}; //How much degrees/s should max stick be?

//Create Servo object, used to send pwm signal
Servo motor1;

float m1;
float throttle;

bool failsafe = false;

/* SbusRx object on UART 1 */
bfs::SbusRx sbus_rx(&mySerial);
std::array<int16_t, bfs::SbusRx::NUM_CH()> sbus_data;



///////////////////////////SETUP/////////////////////////////////////

void initializeServos() {
  //Enable motors
  motor1.attach(_motor1);

<<<<<<< HEAD
  motor1.writeMicroseconds(0);      //Make sure motors are not spinning
=======
  motor1.writeMicroseconds(1000);      //Make sure motors are not spinning
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
>>>>>>> 5d40f2b66fd1f48d29268aee67633c04c1db8475
}
////////////////////////////SetData///////////////////////////////////

//Store Sbus data in array sbus_rx
void getSbus() {
  if (sbus_rx.Read()) {
    if (sbus_rx.lost_frame())return;
    sbus_data = sbus_rx.ch();
  }
  if (sbus_rx.failsafe()) {
    failsafe = true;
<<<<<<< HEAD
    motor1.writeMicroseconds(0);      //Disable motors if receivers looses connection
=======
    motor1.writeMicroseconds(1000);      //Disable motors if receivers looses connection
    motor2.writeMicroseconds(1000);
    motor3.writeMicroseconds(1000);
    motor4.writeMicroseconds(1000);
>>>>>>> 5d40f2b66fd1f48d29268aee67633c04c1db8475
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
<<<<<<< HEAD

  throttle = map(sbus_data[2], 180, 1820, 780, 2000);
=======
  target_roll = map(sbus_data[0], 180, 1820, -axisSensitivity, axisSensitivity);
  target_pitch = map(sbus_data[1], 180, 1820, -axisSensitivity, axisSensitivity);
  target_yaw = map(sbus_data[3], 180, 1820, -axisSensitivity, axisSensitivity);
  throttle = map(sbus_data[2], 180, 1820, 1000, 2000);
>>>>>>> 5d40f2b66fd1f48d29268aee67633c04c1db8475
}

///////////////////////////////////////////////////////////////////////


void applyMotors() {
  if (sbus_rx.lost_frame()) {
    return;
  }
  m1 = throttle;
  m1 = constrain(m1, minValue, maxValue);
  motor1.writeMicroseconds(m1);
}

void setup() {
  int now = millis();

  Serial.begin(115200);
  Serial.println("tekst in" + String(now));

  Serial.println("Starting sbus connection.");
  sbus_rx.Begin(rxpin, txpin);    //Begin Sbus communication

  Serial.println("Initializing servos");
  initializeServos();             //Start connection to motors
  Serial.println("Finished initializing servos");

  Serial.println("Finished setup in: " + String(millis() - now) + " ms");

}

void loop() {

  //////Get system data//////
  getSbus();
  mapInput();

  if (failsafe)return;  //Disable everything if signal is lost or arm button is off.

  //////Apply to motors//////
  applyMotors();
  delay(10);
}
