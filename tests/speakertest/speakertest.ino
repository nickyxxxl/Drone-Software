#include <Tone32.h>
#define BUTTON_PIN 27
#define CHANNEL 0
#define led 2

void adv(unsigned int note, unsigned long del) {
  tone(BUTTON_PIN, note, CHANNEL);
  delay(del);
  noTone(BUTTON_PIN, CHANNEL);
}

void setup() {
  Serial.begin(115200); 
  Serial.print("Started on " + String(xPortGetCoreID()));
  pinMode(led, OUTPUT);
xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 1, NULL,  1); 
  delay(500); 
xTaskCreatePinnedToCore(Task2code, "Task2", 10000, NULL, 1, NULL,  0); 
    delay(500); 
}

void Task1code( void * pvParameters ){
  Serial.print("blinking led running on core ");
  Serial.println(xPortGetCoreID());
  for(;;){
    digitalWrite(led, HIGH);
    delay(300);
    digitalWrite(led, LOW);
    delay(300);
  } 

}
void Task2code( void * pvParameters ){
  Serial.print("SUS running on core ");
  Serial.println(xPortGetCoreID());
  for(;;){
       delay(200);
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
  delay(500);
  }

}
void loop() {
}
