#include <ESP32Servo.h>

#define _motor1 25
#define _motor2 26
#define _motor3 32
#define _motor4 33

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

void setup() {
  Serial.begin(115200);
  Serial.print("Program start\n");
  delay(10000); //ESC startup
  motor1.attach(_motor1);
  motor2.attach(_motor2);
  motor3.attach(_motor3);
  motor4.attach(_motor4);
  
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
      int now = millis();
      Serial.print("Now spinning motor 1!\t");
      while (millis() <= now + 2000) {
        motor1.writeMicroseconds(1300);
        delay(600);
      }motor1.writeMicroseconds(0);

      now = millis();
      Serial.print("Now spinning motor 2!\t");
      while (millis() <= now + 2000) {
        motor2.writeMicroseconds(1300);
        delay(600);
      }motor2.writeMicroseconds(0);

      now = millis();
      Serial.print("Now spinning motor 3!\t");
      while (millis() <= now + 2000) {
        motor3.writeMicroseconds(1300);
        delay(600);
      }motor3.writeMicroseconds(0);

      now = millis();
      Serial.print("Now spinning motor 4!\n");
      while (millis() <= now + 2000) {
        motor4.writeMicroseconds(1300);
        delay(600);
      }motor4.writeMicroseconds(0);
      delay(10000);
}
