#include <DueTimer.h>
#include <LiquidCrystal.h>
#include <SevSeg.h>
#include <ResponsiveAnalogRead.h>
#include <DueFlashStorage.h>


//************* COMMON DEFINITIONS
#define NOTENUM 8 
#define VOICENUM 4
#define VOICEPARAMNUM 6 
#define SUSTLEVELPARAMIDX 3

#define WAVE_SAMPLES 2048
#define SAMPLE_RATE 44100.0
#define SAMPLES_PER_CYCLE 2048
#define SAMPLES_PER_CYCLE_FIXEDPOINT (SAMPLES_PER_CYCLE<<20)
#define TICKS_PER_CYCLE (float)((float)SAMPLES_PER_CYCLE_FIXEDPOINT/(float)SAMPLE_RATE)


//************* FILTER
#define FX_SHIFT 8
#define SHIFTED_1 256
uint8_t q[VOICENUM] = {0.2, 0.2, 0.2, 0.5};
uint8_t q_out = 0.2;
uint8_t f[VOICENUM] = {50, 50, 50, 50};
uint8_t f_out = 62;

int buf0, buf1;
// multiply two fixed point numbers (returns fixed point)
inline
unsigned int ucfxmul(uint8_t a, uint8_t b)
{
  return (((unsigned int)a*b)>>FX_SHIFT);
}
  
// multiply two fixed point numbers (returns fixed point)
inline
int ifxmul(int a, uint8_t b)
{
  return ((a*b)>>FX_SHIFT);
}
  
// multiply two fixed point numbers (returns fixed point)
inline
long fxmul(long a, int b)
{
  return ((a*b)>>FX_SHIFT);
}

unsigned int fb[VOICENUM] = {q[0] + fxmul(q[0], (int)SHIFTED_1 - (f[0] / 128)), q[1] + fxmul(q[1], (int)SHIFTED_1 - (f[1] / 128)), q[2] + fxmul(q[2], (int)SHIFTED_1 - (f[2] / 128)), q[3] + fxmul(q[3], (int)SHIFTED_1 - (f[3] / 128))};
unsigned int fb_out = q_out + fxmul(q_out, (int)SHIFTED_1 - (f_out / 128));

inline
int filter(int in, uint8_t q, uint8_t f, unsigned int fb)
{
  buf0+=fxmul(((in - buf0) + fxmul(fb, buf0-buf1)), f);
  buf1+=ifxmul(buf0-buf1, f); // could overflow if input changes fast
  return buf1;
}

// nSineTable[] is used both in lfo() and audioHandler()
// thats why are they defined here
uint16_t nSineTable[WAVE_SAMPLES];
uint16_t nSquareTable[WAVE_SAMPLES];
uint16_t nSawTable[WAVE_SAMPLES];
uint16_t nTriangleTable[WAVE_SAMPLES];

//************* LFO
uint32_t ulLfoPhaseAccumulator = 0;
volatile uint32_t ulLfoPhaseIncrement = 1000000;
uint16_t lfo(uint32_t in) {
  ulLfoPhaseAccumulator += ulLfoPhaseIncrement;
  if(ulLfoPhaseAccumulator > SAMPLES_PER_CYCLE_FIXEDPOINT) {
      ulLfoPhaseAccumulator -= SAMPLES_PER_CYCLE_FIXEDPOINT;    
    }

  return nSineTable[ulLfoPhaseAccumulator >> 20] * in * 0.00001;
  }  


//************* SYNTH
#define INCREMENT_ONE_FIXEDPOINT 1<<20 // 1048576
int32_t globalOut;
int attackTime[VOICENUM] = {2, 5, 5, 5};        // value < 1 causes clicks (only at high frequency sounds?)
int decayTime[VOICENUM] = {3, 10, 10, 10};
int sustainTime[VOICENUM] = {10, 10, 10, 10};
int sustainLevel[VOICENUM] = {768, 768, 768, 768};
int releaseTime[VOICENUM] = {50, 10, 20, 20};
int voiceFrequency[VOICENUM] = {114, 8, 114, 15};
// the phase accumulator points to the current sample in our wavetable
uint32_t ulPhaseAccumulator[VOICENUM] = {0, 0, 0, 0};
// the phase increment controls the rate at which we move through the wave table
// higher values = higher frequencies
volatile uint32_t ulPhaseIncrement[VOICENUM] = {voiceFrequency[0] * INCREMENT_ONE_FIXEDPOINT, voiceFrequency[1] * INCREMENT_ONE_FIXEDPOINT, voiceFrequency[2] * INCREMENT_ONE_FIXEDPOINT, voiceFrequency[3] * INCREMENT_ONE_FIXEDPOINT};   // 32 bit phase increment, see below
int * voiceParam[VOICEPARAMNUM] = {&attackTime[0], &decayTime[0], &sustainTime[0], &sustainLevel[0], &releaseTime[0], &voiceFrequency[0]};
int32_t envelopeVolume[VOICENUM] = {0, 0, 0, 0}; // the current volume according to the envelope on a scale from 0 to 1023 (10 bits) - needs to be unsigned so we can multiply with it for modulation
unsigned long attackStartTime[VOICENUM] = {0, 0, 0, 0};
unsigned long decayStartTime[VOICENUM] = {0, 0, 0, 0};
unsigned long sustainStartTime[VOICENUM] = {0, 0, 0, 0};
unsigned long releaseStartTime[VOICENUM] = {0, 0, 0, 0};
unsigned char envelopeProgress[VOICENUM] = {0, 0, 0, 0}; // 255 = the envelope is idle
int voiceN = 0;
int voiceParameterN = 0;
uint16_t * soundType[VOICENUM] = {&nSineTable[0], &nSquareTable[0], &nSawTable[0], &nTriangleTable[0]};

void createSineTable() {
  for (uint32_t nIndex = 0; nIndex < WAVE_SAMPLES; nIndex++) {
    // SINE
    nSineTable[nIndex] = (uint16_t)  (((1 + sin(((2.0 * PI) / WAVE_SAMPLES) * nIndex)) * 4095.0) / 2);
  }
}

void createSquareTable(int16_t pw)
{
  static int16_t lastPw = 127; // don't initialize to 0
  if (pw != lastPw)
  {
    for (uint32_t nIndex = 0; nIndex < WAVE_SAMPLES; nIndex++)
    {
      // SQUARE
      if (nIndex <= ((WAVE_SAMPLES / 2) + pw))
        nSquareTable[nIndex] = 0;
      else
        nSquareTable[nIndex] = 4095;
    }
    lastPw = pw;
  }
}

void createSawTable()
{
  for (uint32_t nIndex = 0; nIndex < WAVE_SAMPLES; nIndex++)
  {
    // SAW
    nSawTable[nIndex] = (4095 / WAVE_SAMPLES) * nIndex;
  }
}

void createTriangleTable()
{
  for (uint32_t nIndex = 0; nIndex < WAVE_SAMPLES; nIndex++)
  {
    // Triangle
    if (nIndex < WAVE_SAMPLES / 2)
      nTriangleTable[nIndex] = (4095 / (WAVE_SAMPLES / 2)) * nIndex;
    else
      nTriangleTable[nIndex] = (4095 / (WAVE_SAMPLES / 2)) * (WAVE_SAMPLES - nIndex);
  }
}

void audioHandler() {
  int i;
  uint32_t ulOutput[VOICENUM];  
  for (i = 0; i < VOICENUM; i++) {
    //ulPhaseAccumulator[i] += ulPhaseIncrement[i];   // 32 bit phase increment, see below
    ulPhaseAccumulator[i] += voiceFrequency[i] * INCREMENT_ONE_FIXEDPOINT;   // 32 bit phase increment, see below
    // if the phase accumulator over flows - we have been through one cycle at the current pitch,
    // now we need to reset the grains ready for our next cycle

    if(ulPhaseAccumulator[i] > SAMPLES_PER_CYCLE_FIXEDPOINT)
    {
      // DB 02/Jan/2012 - carry the remainder of the phase accumulator
      ulPhaseAccumulator[i] -= SAMPLES_PER_CYCLE_FIXEDPOINT; 
      //ulPhaseAccumulator[i] = 0;
    }
    
    // get the current sample  
    // filtered:
    //ulOutput = ulOutput + filter((nSineTable[ulPhaseAccumulator[i]>>20] * envelopeVolume[i]), q[i], f[i], fb[i]);
    ulOutput[i] = *(soundType[i] + (ulPhaseAccumulator[i]>>20)) * envelopeVolume[i] >> 10;
    //globalOut = ulOutput[i];  // for debugging (Serial.println())
  }

  int32_t sampleOsc;
  for (int i = 0; i < VOICENUM; i++)
  {
    sampleOsc += ulOutput[i];
  } 
  
  sampleOsc = sampleOsc / VOICENUM;

  
  //globalOut = sampleOsc;

  float mainVolume = 1;                  // 0.00625 = 1/160; 0.0125 = 1/80; 0.025 = 1/40; 0.05 = 1/20; 0.1 = 1/10; 0.2 = 1/5; 0.5 = 1/2
  //sampleOsc = filter(sampleOsc, q_out, f_out, fb_out) * mainVolume;
  sampleOsc = sampleOsc * mainVolume;


  // write to DAC0
  dacc_set_channel_selection(DACC_INTERFACE, 0);
  dacc_write_conversion_data(DACC_INTERFACE, sampleOsc);
  // write to DAC1
  //dacc_set_channel_selection(DACC_INTERFACE, 1);
  //dacc_write_conversion_data(DACC_INTERFACE, sampleOsc);
}

void envelopeHandler() {
  int i;
  for (i = 0; i < VOICENUM; i++) {
    switch (envelopeProgress[i]) {  
      case 0: // ATTACK
        if ((millis() - attackStartTime[i]) > attackTime[i]) {
          decayStartTime[i] = millis();            
          envelopeProgress[i]= 1;
          }
        else {
          envelopeVolume[i] = map(millis(), attackStartTime[i], attackStartTime[i] + attackTime[i], 0, 1023);
          }
        break;
        
      case 1: // DECAY
        if ((millis() - decayStartTime[i]) > decayTime[i]) {
          sustainStartTime[i] = millis();          
          envelopeProgress[i] = 2;
          }    
        else {
          envelopeVolume[i] = map(millis(), decayStartTime[i], decayStartTime[i] + decayTime[i], 1023, voiceParam[SUSTLEVELPARAMIDX][voiceN]);
          }        
        break;

      case 2: // SUSTAIN
        if ((millis() - sustainStartTime[i]) > sustainTime[i]) {
          releaseStartTime[i] = millis();          
          envelopeProgress[i] = 3;
          }    
        else {
          envelopeVolume[i] = sustainLevel[i];
        }
        break;

      case 3: // RELEASE
        if ((millis() - releaseStartTime[i]) > releaseTime[i]) {
          envelopeProgress[i] = 255;
          }    
      else {
        envelopeVolume[i] = map(millis(), releaseStartTime[i], releaseStartTime[i] + releaseTime[i], voiceParam[SUSTLEVELPARAMIDX][voiceN], 0);
        }
        break;

      case 255: // MUTE
        envelopeVolume[i] = 0;
        break;       
      }
    }
  }


//************* SEQ
bool sequences[VOICENUM][NOTENUM] = {{true, false, false, false, false, false, false, false},
                               {false, false, true, false, false, false, false, false},
                               {false, false, false, false, true, false, false, false},
                               {false, false, false, false, false, false, true, false}};     


void trigger(int i) {
  //Serial.println("trigger");  
  attackStartTime[i] = millis();
  envelopeProgress[i] = 0;
  }

int st = 0;
void sequencer() {
  int i, j;
  
  for (j = 0; j < VOICENUM; j++) {
    if (sequences[j][st] == true)
      trigger(j);
    }
    
  if (st == (NOTENUM - 1)) 
    st = 0;
  else
    st++;
  }


//************* BUTTONS
#define DEBOUNCE 10  // button debouncer, how many ms to debounce, 5+ ms is usually plenty
#define NUMBUTTONS sizeof(buttons)
// here is where we define the buttons that we'll use. button "1" is the first, button "6" is the 6th, etc
byte buttons[] = {22, 23, 24, 25, 26, 27, 28, 29, 48, 49, 50, 51, 52, 53, 8}; 
// This handy macro lets us determine how big the array up above is, by checking the size
// we will track if a button is just pressed, just released, or 'currently pressed' 
byte pressed[NUMBUTTONS], justpressed[NUMBUTTONS], justreleased[NUMBUTTONS];
void clearJust()
{
  for (byte index = 0; index < NUMBUTTONS; index++) // when we start, we clear out the "just" indicators
  {
    justreleased[index] = 0;
    justpressed[index] = 0;
  }
}

void buttonsHandler() {
  static byte previousstate[NUMBUTTONS];
  static byte currentstate[NUMBUTTONS];  
  static long lasttime;
  byte index;

  if (millis() < lasttime) { // we wrapped around, lets just try again
     lasttime = millis();
  }
  
  if ((lasttime + DEBOUNCE) > millis()) {
    // not enough time has passed to debounce
    return; 
  }
  // ok we have waited DEBOUNCE milliseconds, lets reset the timer
  lasttime = millis();
  
  for (index = 0; index < NUMBUTTONS; index++) {// when we start, we clear out the "just" indicators
    justreleased[index] = 0;
    justpressed[index] = 0;
     
    currentstate[index] = digitalRead(buttons[index]);   // read the button
    
    if (currentstate[index] == previousstate[index]) {
      if ((pressed[index] == LOW) && (currentstate[index] == LOW)) {        // if not being pressed before and actually being pressed
          // just pressed
          justpressed[index] = 1;          
      }
      else if ((pressed[index] == HIGH) && (currentstate[index] == HIGH)) { // if being pressed before and and acutally being released
          // just released
          justreleased[index] = 1;
      }          
      pressed[index] = !currentstate[index];  // remember, digital HIGH means NOT pressed
    }
    previousstate[index] = currentstate[index];   // keep a running tally of the buttons
  }
}


//************* LEDS
int seqLedState[NOTENUM] = {LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW}; 

void ledsHandler() {
  for (int i = 0; i < NOTENUM; i++) {
    digitalWrite(i + 30, sequences[voiceN][i]);
    }
  }


//************* LCD
int whatDisplay = 0;  // 0 - params; 1 - load; 2 - save
unsigned long whatDisplayTimer = 0;
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void showValue(int val) {
  if (val < 10)
  {
    lcd.print("   ");
    lcd.print(val);
  }
  else if (val < 100)
  {
    lcd.print("  ");
    lcd.print(val);
  }
  else if (val < 1000)
  {
    lcd.print(" ");
    lcd.print(val);
  }
  else
    lcd.print(val);
}

void lcdHandler() {
  lcd.clear();  

  switch (whatDisplay) {
    case 0:
      switch (voiceParameterN) {
        case 0:
          lcd.print("AttTime: ");      
          break;      
        case 1:
          lcd.print("DecTime: ");
          break;      
        case 2:      
          lcd.print("SustTime: ");    
          break;      
        case 3:      
          lcd.print("SustLevel: ");        
          break;      
        case 4:      
          lcd.print("RelTime: ");
          break;      
        case 5:      
          lcd.print("VoiceFreq: ");    
          break;
        }
          
      showValue(voiceParam[voiceParameterN][voiceN]);
    break;
    
    case 1:
      if (millis() - whatDisplayTimer < 500)
        lcd.print("saved");
      else
        whatDisplay = 0;  // params
      break;
    
    case 2:
      if (millis() - whatDisplayTimer < 500)
        lcd.print("loaded");
      else
        whatDisplay = 0;  // params
      break;
    }
  }


//************* POTS
//ResponsiveAnalogRead analog(0, true);
bool locked = false;
unsigned long lastUnlocked = 0;
void potsHandler() {
  //analog.update();    
  static int last = 0;
  int act = analogRead(0); 
  //if (analog.hasChanged()) {
  //  voiceParam[voiceParameterN][voiceN] = analog.getValue();
  //  lcdHandler();
  //  }
  if (locked) {
    if (abs(act - last) > 10) {
      locked = false;
      lastUnlocked = millis();
      }
  }
  else {
    if (millis() - lastUnlocked > 500) {
      locked = true;
      last = act;
      if (voiceParameterN == SUSTLEVELPARAMIDX) // sustainLevel
          voiceParam[voiceParameterN][voiceN] = act;
      else
          voiceParam[voiceParameterN][voiceN] = map(act, 0, 1023, 1, 100);
      lcdHandler();
      }
    }
  }


//************* DEVICE CONTROL
#define SETTINGSSTARTADDR 4
bool stored = false;
DueFlashStorage store;

void playSound() {
  Timer3.attachInterrupt(audioHandler).setFrequency(SAMPLE_RATE).start(); // start the audio interrupt at 44.1kHz      
  Timer4.attachInterrupt(sequencer).setPeriod(400000).start();     
  }

void stopSound() {
  Timer3.stop();
  Timer4.stop();
  }

void upVoice() {
  ++voiceN;  
  if (voiceN > (VOICENUM - 1))
    voiceN = 0;
  }

void downVoice() {
  --voiceN;  
  if (voiceN < 0)
    voiceN = (VOICENUM - 1);
  }

void upVoiceParameter() {
  ++voiceParameterN;  
  if (voiceParameterN > (VOICEPARAMNUM - 1))
    voiceParameterN = 0;
  }

void downVoiceParameter() {
  --voiceParameterN;  
  if (voiceParameterN < 0)
    voiceParameterN = (VOICEPARAMNUM - 1);
  }

void storeSettings() {
  int b[VOICENUM * VOICEPARAMNUM];
  byte b2[VOICENUM * NOTENUM];
  for (int i = 0; i < VOICEPARAMNUM; i++) {
    memcpy(&(b[i*VOICENUM]), voiceParam[i], VOICENUM * sizeof(int));
  }
  memcpy(b2, sequences, sizeof(bool) * VOICENUM * NOTENUM);
  store.write(SETTINGSSTARTADDR, (byte *) b, sizeof(b) * sizeof(int));
  store.write(SETTINGSSTARTADDR + VOICENUM * VOICEPARAMNUM * sizeof(int), b2, sizeof(b2));
  Serial.println("stored");

  whatDisplay = 1;  // save
  whatDisplayTimer = millis();
  }

void loadSettings() {
  for (int i = 0; i < VOICEPARAMNUM; i++) {
    for (int j = 0; j < VOICENUM; j++) {
        memcpy(voiceParam[i], store.readAddress(SETTINGSSTARTADDR + i * VOICENUM * sizeof(int)), VOICENUM * sizeof(int));
    }
  }
  memcpy(sequences, store.readAddress(SETTINGSSTARTADDR + VOICENUM * VOICEPARAMNUM * sizeof(int)), VOICENUM * NOTENUM);
  for (int i = 0; i < VOICEPARAMNUM; i++) {
    for (int j = 0; j < VOICENUM; j++) {
      Serial.println(voiceParam[i][j]);
      }
    Serial.println();      
    }
  Serial.println("loaded");

  whatDisplay = 2;  // load
  whatDisplayTimer = millis();
  }

void deviceControlHandler() {
  // temp sequencer buttons
  for (int i = 0; i < NOTENUM; i++) {       
    if (pressed[i] && sequences[voiceN][i]) {         // justpressed works equally bad here
      sequences[voiceN][i] = false;
      seqLedState[i] = LOW;
    }
    else if (pressed[i] && !sequences[voiceN][i]) {   // justpressed works equally bad here
      sequences[voiceN][i] = true;
      seqLedState[i] = HIGH;      
    }
  }

  // temp settings store
  if (justpressed[14]) {// or justpressed? both work
    if (stored == true) {
      loadSettings();
      stored = false;
    }
    else {
      storeSettings();
      stored = true;
    }
  }  
  
  // temp up down voice parameter
  if (justpressed[8]) {// or justpressed? both work
    upVoiceParameter();
    lcdHandler();
  }  
  if (justpressed[9]) {// or justpressed? both work
    downVoiceParameter();
    lcdHandler();
  }

  // temp up down voice
  if (justpressed[10]) // or justpressed? both work
    upVoice();  
  if (justpressed[11]) // or justpressed? both work
    downVoice();

  // temp play stop 
  if (pressed[12]) // or justpressed? both work
    stopSound();
  if (pressed[13]) // or justpressed? both work
    playSound();

  clearJust();  
  }

//************* MAIN
SevSeg sevseg;

void setup() {
  Serial.begin(9600);  

   byte numDigits = 1;   
   byte digitPins[] = {6, 7};
   byte segmentPins[] = {38, 39, 40, 41, 42, 43, 44, 45};
   bool resistorsOnSegments = true; // Use 'true' if on digit pins
   byte hardwareConfig = COMMON_CATHODE; // See README.md for options
   sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments);
   sevseg.setBrightness(90);

  for (int i = 0; i < 5; i++) {  
    lcd.begin(16, 2);
    lcd.print("ArduinoSynth");  
    lcd.setCursor(0, 1);
    lcd.print("v 0.1");
    lcd.setCursor(14, 1);
    lcd.print(5 - i);
    lcd.print("s");
    delay(1000);
    lcd.clear();
  }

  // buttons
  // settings store button  
  pinMode(8, INPUT_PULLUP);
  
  // Make input & enable pull-up resistors on switch pins
  for (byte i = 0; i < NUMBUTTONS; i ++)
    pinMode(buttons[i], INPUT_PULLUP);

  // up down buttons for voice change
  pinMode(50, INPUT_PULLUP);
  pinMode(51, INPUT_PULLUP);

  // play stop buttons
  pinMode(52, INPUT_PULLUP);
  pinMode(53, INPUT_PULLUP);


  // LEDs
  for (int i = 0; i < NOTENUM; i++) {
    pinMode(i + 30, OUTPUT);
  }  

  uint8_t codeRunningForTheFirstTime = store.read(0);
  if (codeRunningForTheFirstTime) {
    Serial.println("yes");
    storeSettings();
    store.write(0, 0);     
  }

  loadSettings();
    
  createSineTable();
  createSquareTable(100);  
  createSawTable();
  createTriangleTable();

  analogWrite(DAC0, 0);
  analogWrite(DAC1, 0);

  playSound();
}

void loop() { 
  envelopeHandler();  
  buttonsHandler();    
  ledsHandler();   
  potsHandler();
  deviceControlHandler();
  sevseg.setNumber(voiceN, 0);  
  sevseg.refreshDisplay();
}
