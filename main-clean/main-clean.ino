#include <MPU6050_tockn.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <sbus.h>
#include <Adafruit_INA219.h>
#include <Tone32.h>

///////////////////////////User defined/////////////////////////////////////

//Define the pins for each motor
#define _motor1 25
#define _motor2 26
#define _motor3 32
#define _motor4 33

//Define pin for sbus rx and tx channel (we only care about rx)
const int8_t rxpin {16};
const int8_t txpin {17}; //We don't use this, set it to whatever.
HardwareSerial mySerial(1);

const int minValue {800};     //min max values ESC
const int maxValue {2000};

//PID tuning
const float kp_roll {1.3},
      ki_roll {0.04},
      kd_roll {18.0};
const int max_roll = 400;

const float kp_pitch {1.3},
      ki_pitch {0.04},
      kd_pitch {18.0};
const int max_pitch = 400;

const float kp_yaw {4.0},
      ki_yaw {0.02},
      kd_yaw {0.0};
const int max_yaw = 400;

const float axisSensitivity {100}; //How much degrees/s should max stick be?

//////////////////////////////////////////////////////////////////////////

MPU6050 mpu6050(Wire);
//Adafruit_INA219 ina219;

//Create Servo object, used to send pwm signal
Servo motor1;                       //  4 ----- 2
Servo motor2;                       //   |  ^  |
Servo motor3;                       //   |     |
Servo motor4;                       //  3 ----- 1

//Operational variables
bool failsafe = false;
float deltaT;

float currentTime;
float lastTime;

//Gyro data
float gyro_roll, gyro_pitch, gyro_yaw;

//Target values user input
float target_roll, target_pitch, target_yaw, throttle;


//Output values of PID
float previous_i_roll {},
      previous_error_roll {},
      PID_output_roll {};

float previous_i_pitch {},
      previous_error_pitch {},
      PID_output_pitch     {};

float previous_i_yaw {},
      previous_error_yaw {},
      PID_output_yaw     {};

float m1;
float m2;
float m3;
float m4;


/* SbusRx object on UART 1 */
bfs::SbusRx sbus_rx(&mySerial);
/* Array for storing SBUS data */
std::array<int16_t, bfs::SbusRx::NUM_CH()> sbus_data;



///////////////////////////SETUP/////////////////////////////////////

void initializeServos() {
  //Enable motors
  motor1.attach(_motor1);
  motor2.attach(_motor2);
  motor3.attach(_motor3);
  motor4.attach(_motor4);

  motor1.writeMicroseconds(0);      //Make sure motors are not spinning
  motor2.writeMicroseconds(0);
  motor3.writeMicroseconds(0);
  motor4.writeMicroseconds(0);
}
///////////////////////////////////////////////////////////////////////

////////////////////////////SetData///////////////////////////////////

//Store Sbus data in array sbus_rx
void getSbus() {
  if (sbus_rx.lost_frame()) {
    return;
  }
  if (sbus_rx.Read()) {
    sbus_data = sbus_rx.ch();
  }
}


void getGyro() {
  mpu6050.update();
  gyro_roll = mpu6050.getGyroY();
  gyro_pitch = mpu6050.getGyroZ();
  gyro_yaw = mpu6050.getGyroX();
}

///////////////////////////////////////////////////////////////////////

//function to map a float from range (in_min, in_out) to (out_min, out_max)
float mapFloat(float input, float in_min, float in_max, float out_min, float out_max) {
  return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
  target_roll = map(sbus_data[0], 180, 1820, -axisSensitivity, axisSensitivity);
  target_pitch = map(sbus_data[1], 180, 1820, -axisSensitivity, axisSensitivity);
  target_yaw = map(sbus_data[3], 180, 1820, -axisSensitivity, axisSensitivity);
  throttle = map(sbus_data[2], 180, 1820, 800, 1750);
}


//PID for each axis
void calculatePID_Roll() {

  float error {target_roll - gyro_roll};

  float pTerm {error};
  float iTerm {previous_i_roll + error * deltaT};
  float dTerm {(previous_error_roll - error) / deltaT};

  PID_output_roll = (pTerm * kp_roll) + (iTerm * ki_roll) + (dTerm * kd_roll);

  previous_error_roll = error;
  previous_i_roll = iTerm;
}

void calculatePID_Pitch() {

  float error {target_pitch - gyro_pitch};

  float pTerm {error};
  float iTerm {previous_i_pitch + error * deltaT};
  float dTerm {(previous_error_pitch - error) / deltaT};

  PID_output_pitch = (pTerm * kp_pitch) + (iTerm * ki_pitch) + (dTerm * kd_pitch);

  previous_error_pitch = error;
  previous_i_pitch = iTerm;
}

void calculatePID_Yaw() {

  float error {target_yaw - gyro_yaw};

  float pTerm {error};
  float iTerm {previous_i_yaw + error * deltaT};
  float dTerm {(previous_error_yaw - error) / deltaT};

  PID_output_yaw = (pTerm * kp_yaw) + (iTerm * ki_yaw) + (dTerm * kd_yaw);

  previous_error_yaw = error;
  previous_i_yaw = iTerm;
}

///////////////////////////////////////////////////////////////////////


void applyMotors() {
  if (sbus_rx.lost_frame()) {                                      //ignore control if frame lost
    return;
  }

  if (sbus_data[4] < 1200 || sbus_rx.failsafe()) {                                       //arm
    motor1.writeMicroseconds(0);
    motor2.writeMicroseconds(0);
    motor3.writeMicroseconds(0);
    motor4.writeMicroseconds(0);
    return;
  }
  
  if (sbus_data[5] < 1200){                                        //direct mode
    m1 = throttle;
    m2 = throttle;
    m3 = throttle;
    m4 = throttle;
  } else if (sbus_data[5] > 900 && sbus_data[5] < 1800) {         //PID mode
    m1 = throttle - PID_output_roll - PID_output_pitch - PID_output_yaw;
    m2 = throttle - PID_output_roll + PID_output_pitch + PID_output_yaw;
    m3 = throttle + PID_output_roll - PID_output_pitch + PID_output_yaw;
    m4 = throttle + PID_output_roll + PID_output_pitch - PID_output_yaw;
  }
  constrain(m1, minValue, maxValue);  //constrain between min and max esc value
  constrain(m2, minValue, maxValue);
  constrain(m3, minValue, maxValue);
  constrain(m4, minValue, maxValue);

  motor1.writeMicroseconds(m1);
  motor2.writeMicroseconds(m2);
  motor3.writeMicroseconds(m3);
  motor4.writeMicroseconds(m4);
}

void setup() {
  delay(1000);
  
  sbus_rx.Begin(rxpin, txpin);    //Begin Sbus communication

  Wire.begin();
  mpu6050.begin();                //Start gyro communication
  
  mpu6050.calcGyroOffsets(true);  //true if you want debug output, blank if not !!Do not move during this period!!
  initializeServos();             //Start connection to motors
}

void loop() {
  //////Get system data//////
  getSbus();
  getGyro();
  mapInput();

  //////Calculate PID//////
  if (sbus_data[5] > 900 && sbus_data[5] < 1800) {
    currentTime = millis();
    deltaT = (currentTime - lastTime) * 0.001; //*0.001 to go from ms to s
    calculatePID_Roll();
    calculatePID_Pitch();
    calculatePID_Yaw();
    lastTime = currentTime;
  }
    
  //////Apply to motors//////
  applyMotors();
  delay(6);
}
