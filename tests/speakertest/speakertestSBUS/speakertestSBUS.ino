#include <sbus.h>

const int8_t rxpin {16};
const int8_t txpin {17}; //We don't use this, set it to whatever.
HardwareSerial mySerial(1);
#define  speaker 16

enum Notes { note_c = 261, note_cs = 277, note_d = 293, note_ds = 311, note_e = 329, note_f = 349, note_fs = 370, note_g = 392, note_gs = 415, note_a = 440, note_as = 466, note_b = 494};
float octave = 0.5;
bool firstTime{true};
unsigned long songStart;


/* SbusRx object on UART 1 */
bfs::SbusRx sbus_rx(&mySerial);
/* Array for storing SBUS data */
std::array<int16_t, bfs::SbusRx::NUM_CH()> sbus_data;


void getSbus(){
  if (sbus_rx.Read()){
    sbus_data = sbus_rx.ch();
  }
}


void setup() {
    Serial.begin(115200);
    Serial.println("starting speaker");
    ledcSetup(speaker, 0, 8); //setup speaker
    Serial.println("starting sbus");
    sbus_rx.Begin(rxpin, txpin);    //Begin Sbus communication
}

void loop() {

  Serial.println(":)");
  if(sbus_data[5] >= 1600)music();
  else{songStart = 0; firstTime = true; ledcWriteTone(speaker, 0);}

}


  void music(){

    //bpm = 94, halve = 1276ms, kwart = 638ms, achste = 319, triool = 213 ms, zestiende = 160
    if(firstTime){
      songStart = millis();
      ledcWriteTone(speaker, note_c * octave * octave); return;           //1
    }else{
    unsigned long beat = millis() - songStart;
    if(beat > 638 && beat < 957) {ledcWriteTone(speaker, note_c); return;}        
    else if(beat >= 957 && beat < 1276) {ledcWriteTone(speaker, note_ds); return;}     //1
    else if(beat >= 1276 && beat < 1595) {ledcWriteTone(speaker, note_f); return;}     //1
    else if(beat >= 1595 && beat < 1914) {ledcWriteTone(speaker, note_fs); return;}    //1
    else if(beat >= 1914 && beat < 2233) {ledcWriteTone(speaker, note_f); return;}     //1
    else if(beat >= 2233 && beat < 2552) {ledcWriteTone(speaker, note_ds); return;}    //Measure 1 
    else if(beat >= 2552 && beat < 3509) {ledcWriteTone(speaker, note_c); return;}     //2
    else if(beat >= 3509 && beat < 3669) {ledcWriteTone(speaker, note_as); return;}    //2
    else if(beat >= 3669 && beat < 3829) {ledcWriteTone(speaker, note_d); return;}     //2
    else if(beat >= 3829 && beat < 5105) {ledcWriteTone(speaker, note_c); return;}     //2
    else if(beat >= 5105 && beat < 5425) {ledcWriteTone(speaker, note_g * octave); return;}    //Measure 2
    else if(beat >= 5425 && beat < 6063) {ledcWriteTone(speaker, note_c * octave * octave); return;}   //3
    else if(beat >= 6063 && beat < 6382) {ledcWriteTone(speaker, note_c); return;}     //3
    else if(beat >= 6382 && beat < 6701) {ledcWriteTone(speaker, note_ds); return;}    //3
    else if(beat >= 6701 && beat < 7020) {ledcWriteTone(speaker, note_f); return;}     //3
    else if(beat >= 7020 && beat < 7339) {ledcWriteTone(speaker, note_fs); return;}    //3
    else if(beat >= 7339 && beat < 7658) {ledcWriteTone(speaker, note_f); return;}     //3
    else if(beat >= 7658 && beat < 7977) {ledcWriteTone(speaker, note_ds); return;}    //Measure 3
    else if(beat >= 7977 && beat < 9253) {ledcWriteTone(speaker, note_fs); return;}    //4
    else if(beat >= 9253 && beat < 9466) {ledcWriteTone(speaker, note_fs); return;}    //4
    else if(beat >= 9466 && beat < 9679) {ledcWriteTone(speaker, note_f); return;}     //4
    else if(beat >= 9679 && beat < 9892) {ledcWriteTone(speaker, note_ds); return;}    //4
    else if(beat >= 9892 && beat < 10105) {ledcWriteTone(speaker, note_fs); return;}   //4
    else if(beat >= 10105 && beat < 10318) {ledcWriteTone(speaker, note_f); return;}   //4
    else if(beat >= 10318 && beat < 10531) {ledcWriteTone(speaker, note_ds); return;}  //4
    else if(beat >= 10531 && beat < 11807) {ledcWriteTone(speaker, note_c * octave * octave); return;}    //Measure 4
    else(firstTime = true);
    }

  }
