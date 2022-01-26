#include <Tone32.h>
#define BUTTON_PIN 27
#define CHANNEL 0

void setup() {
  Serial.begin(115200);
  Serial.println("starting speaker");
}

void loop() {
  delay(200);
  adv(NOTE_C2, 638);
  adv(NOTE_C4, 319);
  adv(NOTE_DS4, 319);
  adv(NOTE_F4, 319);
  adv(NOTE_FS4, 319);
  adv(NOTE_F4, 319);
  adv(NOTE_DS4, 319);
  
  adv(NOTE_C4, 957);
  adv(NOTE_E4, 160);
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

void adv(unsigned int note, unsigned long del) {
  tone(BUTTON_PIN, note, CHANNEL);
  delay(del);
  noTone(BUTTON_PIN, CHANNEL);
}
