#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);

//Gyro data
float gyro_roll, gyro_pitch, gyro_yaw;

void getGyro() {
  mpu6050.update();
  gyro_roll = mpu6050.getGyroX();
  gyro_pitch = mpu6050.getGyroY();
  gyro_yaw = mpu6050.getGyroZ();
}

void setup() {

  int now = millis();
  Serial.begin(115200);
  while (!Serial) {}
  Serial.print("Test1!");

  Serial.print("Starting gyro connection.");
  Wire.begin();
  mpu6050.begin();                //Start gyro communication
  Serial.print("About to start callibration!");
  delay(1000);                   //Give user time to put drone down before calibration
  mpu6050.calcGyroOffsets(true);  //true if you want debug output, blank if not !!Do not move during this period!!
  Serial.print("Finished calibrating gyro.");
  
  Serial.print("Finished setup in: ");
  Serial.print(millis() - now);
  Serial.print(" ms");
}

void loop() {
  getGyro();
  Serial.print("roll:" + String(gyro_roll) + "  ");
  Serial.print("pitch:" + String(gyro_pitch) + "  ");
  Serial.print("yaw:" + String(gyro_yaw) + "\n");
  Serial.print("\n");
  delay(150);
}
