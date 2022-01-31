#include <MPU6050_tockn.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <sbus.h>
#include <Adafruit_INA219.h>
#include <Tone32.h>

///////////////////////////User defined/////////////////////////////////////

//Define the pins for each motor
#define _motor1 34
#define _motor2 35
#define _motor3 32
#define _motor4 33

#define speaker 27
#define channel 0

//Define pin for sbus rx and tx channel (we only care about rx)
const int8_t rxpin {
  16
};
const int8_t txpin {17}; //We don't use this, set it to whatever.
HardwareSerial mySerial(1);

const uint8_t buttonUnpressed {1400};   //SBUS value when button is NOT pressed
const uint8_t buttonPressed {
  1600
};     //value when PRESSED
bool lastArmButtonState{0};
bool armButtonState{0};

const int minValue {1000};     //min max values ESC
const int maxValue {
  2000
};

//PID tuning
const float kp_roll {1},
      ki_roll {1},
      kd_roll {1};

const float kp_pitch {1},
      ki_pitch {1},
      kd_pitch {1};

const float kp_yaw {1},
      ki_yaw {1},
      kd_yaw {1};

const float axisSensitivity {100}; //How much degrees/s should max stick be?

void FakingTasks(void * pvParameters);
void Armsound(void * pvParameters);


TaskHandle_t TaskHandle_2;  //Amogus
TaskHandle_t TaskHandle_3;  //Coc
//////////////////////////////////////////////////////////////////////////

MPU6050 mpu6050(Wire);
Adafruit_INA219 ina219;

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
  if (sbus_rx.Read()) {
    sbus_data = sbus_rx.ch();

  }
  if (sbus_rx.failsafe() || sbus_data[4] <= buttonUnpressed) {
    failsafe = true;
    motor1.writeMicroseconds(0);      //Disable motors if receivers looses connection
    motor2.writeMicroseconds(0);
    motor3.writeMicroseconds(0);
    motor4.writeMicroseconds(0);
  }
  else failsafe = false;

}


void getGyro() {
  mpu6050.update();
  gyro_roll = mpu6050.getGyroX();
  gyro_pitch = mpu6050.getGyroY();
  gyro_yaw = mpu6050.getGyroZ();
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
  throttle = map(sbus_data[2], 180, 1820, 1000, 1750);
}

void playArmSound() {
  if (sbus_data[4] <= buttonUnpressed) {
    armButtonState = false;
    if (armButtonState != lastArmButtonState) {
      lastArmButtonState = armButtonState;
      xTaskCreatePinnedToCore(Armsound, "Task3", 10000, NULL, 1, &TaskHandle_3,  0);
    }
  }
  if (sbus_data[4] >= buttonPressed) {
    armButtonState = true;
    if (armButtonState != lastArmButtonState) {
      lastArmButtonState = armButtonState;
      xTaskCreatePinnedToCore(Armsound, "Task3", 10000, NULL, 1, &TaskHandle_3,  0);
    }
  }
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

  m1 = throttle - PID_output_roll - PID_output_pitch - PID_output_yaw;
  m2 = throttle - PID_output_roll + PID_output_pitch + PID_output_yaw;
  m3 = throttle + PID_output_roll - PID_output_pitch + PID_output_yaw;
  m4 = throttle + PID_output_roll + PID_output_pitch - PID_output_yaw;

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

  int now = millis();

  Serial.println("tekst in" + String(now));
  Serial.begin(115200);
  Serial.println("Test2!");
  while (!Serial) {}
  Serial.println("Test1!");

  Serial.println("Starting sbus connection.");
  sbus_rx.Begin(rxpin, txpin);    //Begin Sbus communication

  Serial.println("Starting gyro connection.");
  Wire.begin();
  mpu6050.begin();                //Start gyro communication
  Serial.println("About to start callibration!");
  delay(5000);                   //Give user time to put drone down before calibration
  mpu6050.calcGyroOffsets(true);  //true if you want debug output, blank if not !!Do not move during this period!!
  Serial.println("Finished calibrating gyro.");

  Serial.println("Initializing current sensor.");
  while (!ina219.begin());
  ledcSetup(speaker, 0, 8);
  Serial.println("Finished initializing current sensor");

  Serial.println("Initializing servos");
  initializeServos();             //Start connection to motors
  Serial.println("Finished initializing servos");

  //Offload voltage check and other TASKS to 2nd core
  Serial.println("Initializing second core.");
  xTaskCreatePinnedToCore(FakingTasks, "Task2", 10000, NULL, 1, &TaskHandle_2,  0);

  Serial.println("Finished setup in: " + String(millis() - now) + " ms");

}

void loop() {

  //////Get system data//////
  getSbus();
  playArmSound();
  getGyro();
  mapInput();

  //////Calculate PID//////
  currentTime = millis();
  deltaT = (currentTime - lastTime) * 0.001; //*0.001 to go from ms to s
  calculatePID_Roll();
  calculatePID_Pitch();
  calculatePID_Yaw();
  lastTime = currentTime;

  if (failsafe)return;  //Disable everything if signal is lossed or arm button is off.

  //////Apply to motors//////
  applyMotors();
}

////////////////////////////2ND CORE////////////////////////////////


void coc();
void Armsound(void * pvParameters) {
  //Setup
  vTaskDelete(&TaskHandle_2);
  Serial.println("Playing Armsound on: " + String(xPortGetCoreID()));
  coc();
  xTaskCreatePinnedToCore(FakingTasks, "Task2", 10000, NULL, 1, &TaskHandle_2,  0);
  vTaskDelete(NULL);

}

void FakingTasks(void * pvParameters) {
  //Setup
  Serial.println("Fixing wires");
  Serial.println("Bodies in electrical: " + String(xPortGetCoreID()));
  bool batteryWarning = false;

  //Loop
  while (true) {
    batteryWarning = voltageWarning();
    if (batteryWarning || sbus_data[5] <= buttonUnpressed) return;
    amogus();
  }
}


bool voltageWarning() {
  bool warning{false};

  if (ina219.getBusVoltage_V() <= 6.5) {
    tone(speaker, 3800, channel);
    warning = true;
  }
  return warning;
}

void adv(unsigned int note, unsigned long del) {
  tone(speaker, note, channel);
  delay(del);
  noTone(speaker, channel);
}



void amogus() {
  adv(NOTE_C2, 638);
  adv(NOTE_C4, 319);
  adv(NOTE_DS4, 319);
  adv(NOTE_F4, 319);
  adv(NOTE_FS4, 319);
  adv(NOTE_F4, 319);
  adv(NOTE_DS4, 319);

  adv(NOTE_C4, 957);
  adv(NOTE_AS3, 160);
  adv(NOTE_D4, 160);
  adv(NOTE_C4, 957);
  adv(NOTE_G3, 320);

  adv(NOTE_C2, 638);
  adv(NOTE_C4, 319);
  adv(NOTE_DS4, 319);
  adv(NOTE_F4, 319);
  adv(NOTE_FS4, 319);
  adv(NOTE_F4, 319);
  adv(NOTE_DS4, 319);

  adv(NOTE_FS4, 1276);
  adv(NOTE_FS4, 213);
  adv(NOTE_F4, 213);
  adv(NOTE_DS4, 213);
  adv(NOTE_FS4, 213);
  adv(NOTE_F4, 213);
  adv(NOTE_DS4, 213);
  adv(NOTE_C2, 1276);
}

void coc() {
  adv(NOTE_CS4, 136);
  adv(NOTE_FS4, 136);
  adv(NOTE_GS4, 136);
  adv(NOTE_CS5, 136);
  adv(NOTE_FS5, 136);
  delay(408);
  adv(NOTE_DS4, 272);
  adv(NOTE_FS4, 272);
  adv(NOTE_DS4, 272);
  adv(NOTE_GS4, 272);
}
