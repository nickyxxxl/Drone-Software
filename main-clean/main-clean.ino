#include <MPU6050_tockn.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <sbus.h>

///////////////////////////User defined/////////////////////////////////////

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

//Define the pins for each motor
#define _motor1 25
#define _motor2 26
#define _motor3 32
#define _motor4 33

//Define pin for sbus rx and tx channel (we only care about rx)
const int8_t rxpin {16};
const int8_t txpin {17}; //We don't use this, set it to whatever.
HardwareSerial mySerial(1);

const int minValue {1000};     //min max values ESC
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

const float axisSensitivity {1000}; //How much degrees/s should max stick be?

//////////////////////////////////////////////////////////////////////////

MPU6050 mpu6050(Wire);

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



void initializeServos(){
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  motor1.setPeriodHertz(50);
  motor2.setPeriodHertz(50);
  motor3.setPeriodHertz(50);
  motor4.setPeriodHertz(50);
  motor1.attach(_motor1, 1000, 2000);
  motor2.attach(_motor2, 1000, 2000);
  motor3.attach(_motor3, 1000, 2000);
  motor4.attach(_motor4, 1000, 2000);
}

////////////////////////////SetData///////////////////////////////////

//Store Sbus data in array sbus_rx
void getSbus() {
  
  if (sbus_rx.Read()) {
    if (sbus_rx.lost_frame())return;
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
  throttle = map(sbus_data[2], 180, 1820, minValue, maxValue);
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
    
  if (sbus_data[5] < 900){                                        //direct mode
    m1 = throttle - target_roll - target_pitch - target_yaw;
    m2 = throttle - target_roll + target_pitch + target_yaw;
    m3 = throttle + target_roll - target_pitch + target_yaw;
    m4 = throttle + target_roll + target_pitch - target_yaw;
  } else{                                                        //PID mode
    m1 = throttle - PID_output_roll - PID_output_pitch - PID_output_yaw;
    m2 = throttle - PID_output_roll + PID_output_pitch + PID_output_yaw;
    m3 = throttle + PID_output_roll - PID_output_pitch + PID_output_yaw;
    m4 = throttle + PID_output_roll + PID_output_pitch - PID_output_yaw;
  }
  
    m1 = constrain(m1, minValue, maxValue);  //constrain between min and max esc value
    m2 = constrain(m2, minValue, maxValue);
    m3 = constrain(m3, minValue, maxValue);
    m4 = constrain(m4, minValue, maxValue);

  Serial.print("1: " + String(m1) + "\t" + "2: " + String(m2) + "\t" + "3: " + String(m3) + "\t" + "4: " + String(m4) + "\n");
  
  if (sbus_data[4] < 1200 || sbus_rx.failsafe()) {      //unarmed or failsave
    analogWrite(_motor1, 1000);
    motor1.write(1000);
    analogWrite(_motor2, 1000);
    motor2.write(1000);
    analogWrite(_motor3, 1000);
    motor3.write(1000);
    analogWrite(_motor4, 1000);
    motor4.write(1000);
  } else {
    analogWrite(_motor1, m1);
    motor1.write(m1);
    analogWrite(_motor2, m2);
    motor1.write(m2);
    analogWrite(_motor3, m3);
    motor1.write(m3);
    analogWrite(_motor4, m4);
    motor1.write(m4);
  }
}

void setup() {
  delay(1000);
  Serial.begin(115200);
  
  sbus_rx.Begin(rxpin, txpin);    //Begin Sbus communication

  Wire.begin();
  mpu6050.begin();                //Start gyro communication
  
  mpu6050.setGyroOffsets(-17.10, 2.69, 0.63);  //true if you want debug output, blank if not !!Do not move during this period!!

  initializeServos();
}

void loop() {
  //////Get system data//////
  getSbus();
  if (sbus_rx.lost_frame()) {                                      //ignore control if frame lost
    Serial.println("frame lost!");
    delay(3);
    return;
  }
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
}
