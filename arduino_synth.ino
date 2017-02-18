#include <DueTimer.h>
#include <LiquidCrystal.h>

#define SAMPLE_RATE 44100.0
#define SAMPLES_PER_CYCLE 2048
#define SAMPLES_PER_CYCLE_FIXEDPOINT (SAMPLES_PER_CYCLE<<20)
#define TICKS_PER_CYCLE (float)((float)SAMPLES_PER_CYCLE_FIXEDPOINT/(float)SAMPLE_RATE)

#define WAVE_SAMPLES 2048

#define N 8 

#define VOICENUM 4
int32_t globalOut;

//int attackTime[VOICENUM] = {50, 30, 60, 80};
//int decayTime[VOICENUM] = {50, 30, 60, 70};
//int sustainTime[VOICENUM] = {200, 300, 300, 150};
//int sustainLevel = 400;
//int releaseTime[VOICENUM] = {50, 30, 40, 50};
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
//volatile uint32_t ulPhaseIncrement[VOICENUM] = {60000, 30000, 10000, 20000};   // 32 bit phase increment, see below
//volatile uint32_t ulPhaseIncrement[VOICENUM] = {120000000, 8000000, 12000000, 15000000};   // 32 bit phase increment, see below
#define INCREMENT_ONE_FIXEDPOINT 1048576
volatile uint32_t ulPhaseIncrement[VOICENUM] = {voiceFrequency[0] * INCREMENT_ONE_FIXEDPOINT, voiceFrequency[1] * INCREMENT_ONE_FIXEDPOINT, voiceFrequency[2] * INCREMENT_ONE_FIXEDPOINT, voiceFrequency[3] * INCREMENT_ONE_FIXEDPOINT};   // 32 bit phase increment, see below
int * voiceParam[6] = {&attackTime[0], &decayTime[0], &sustainTime[0], &sustainLevel[0], &releaseTime[0], &voiceFrequency[0]};
int32_t envelopeVolume[VOICENUM] = {0, 0, 0, 0}; // the current volume according to the envelope on a scale from 0 to 1023 (10 bits) - needs to be unsigned so we can multiply with it for modulation
unsigned long attackStartTime[VOICENUM] = {0, 0, 0, 0};
unsigned long decayStartTime[VOICENUM] = {0, 0, 0, 0};
unsigned long sustainStartTime[VOICENUM] = {0, 0, 0, 0};
unsigned long releaseStartTime[VOICENUM] = {0, 0, 0, 0};
unsigned char envelopeProgress[VOICENUM] = {0, 0, 0, 0}; // 255 = the envelope is idle
int voiceN = 0;
int voiceParameterN = 0;
int ctrl = 0;
int val = 0;
//bool sequences[VOICENUM][8] = {{true, false, false, false, true, false, false, false}, 
//                               {false, false, true, false, false, false, false, false},
//                               {true, false, true, true, false, true, false, false},
//                               {false, true, false, false, true, false, true, false}};
//bool sequences[VOICENUM][8] = {{true, true, true, true, true, true, true, true}, 
//                               {false, true, false, true, false, true, false, true},
//                               {false, true, true, true, false, true, true, true},
//                               {true, false, false, false, true, false, false, false}};
/*bool sequences[VOICENUM][8] = {{true, false, false, false, true, false, true, false}, 
                               {false, false, true, true, true, true, true, true},
                               {false, false, false, false, true, true, false, false},
                               {false, false, false, false, false, false, true, true}};*/
/*bool sequences[VOICENUM][8] = {{true, true, true, true, true, true, true, true},
                               {false, false, false, false, false, false, false, true},
                               {false, false, false, false, false, false, false, true},
                               {false, false, false, false, false, false, false, true}};*/
/*bool sequences[VOICENUM][8] = {{true, true, false, false, false, false, false, false},
                               {false, false, true, true, false, false, false, false},
                               {false, false, false, false, true, true, false, false},
                               {false, false, false, false, false, false, true, true}};*/                               
bool sequences[VOICENUM][8] = {{false, false, false, false, false, false, false, false},
                               {false, false, false, false, false, false, false, false},
                               {false, false, false, false, false, false, false, false},
                               {false, false, false, false, false, false, false, false}};                              
#define DEBOUNCE 10  // button debouncer, how many ms to debounce, 5+ ms is usually plenty
// here is where we define the buttons that we'll use. button "1" is the first, button "6" is the 6th, etc
byte buttons[] = {22, 23, 24, 25, 26, 27, 28, 29, 48, 49, 50, 51, 52, 53}; // the analog 0-5 pins are also known as 14-19
// This handy macro lets us determine how big the array up above is, by checking the size
#define NUMBUTTONS sizeof(buttons)
// we will track if a button is just pressed, just released, or 'currently pressed' 
byte pressed[NUMBUTTONS], justpressed[NUMBUTTONS], justreleased[NUMBUTTONS];
  
//int seqButtonState[N] = {LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW}; 
int seqLedState[N] = {LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW}; 
//long lastSeqDebounceTime[N] = {0, 0, 0, 0, 0, 0, 0, 0};  
//long debounceDelay = 50;    
//bool sendOn[N] = {true, true, true, true, true, true, true, true};
//bool sendOff[N] = {false, false, false, false, false, false, false, false};

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

uint16_t nSineTable[WAVE_SAMPLES];
uint16_t nSquareTable[WAVE_SAMPLES];
uint16_t nSawTable[WAVE_SAMPLES];
uint16_t nTriangleTable[WAVE_SAMPLES];

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

#define FX_SHIFT 8
#define SHIFTED_1 256

uint8_t q[VOICENUM] = {0.2, 0.2, 0.2, 0.5};
uint8_t q_out = 0.2;
uint8_t f[VOICENUM] = {50, 50, 50, 50};
uint8_t f_out = 62;
unsigned int fb[VOICENUM] = {q[0] + fxmul(q[0], (int)SHIFTED_1 - (f[0] / 128)), q[1] + fxmul(q[1], (int)SHIFTED_1 - (f[1] / 128)), q[2] + fxmul(q[2], (int)SHIFTED_1 - (f[2] / 128)), q[3] + fxmul(q[3], (int)SHIFTED_1 - (f[3] / 128))};
unsigned int fb_out = q_out + fxmul(q_out, (int)SHIFTED_1 - (f_out / 128));

int buf0,buf1;

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

inline
int filter(int in, uint8_t q, uint8_t f, unsigned int fb)
{
  //setPin13High();
  buf0+=fxmul(((in - buf0) + fxmul(fb, buf0-buf1)), f);
  buf1+=ifxmul(buf0-buf1, f); // could overflow if input changes fast
  //setPin13Low();
  return buf1;
}

uint32_t ulLfoPhaseAccumulator = 0;
volatile uint32_t ulLfoPhaseIncrement = 1000000;
uint16_t lfo(uint32_t in) {
  ulLfoPhaseAccumulator += ulLfoPhaseIncrement;
  if(ulLfoPhaseAccumulator > SAMPLES_PER_CYCLE_FIXEDPOINT) {
      ulLfoPhaseAccumulator -= SAMPLES_PER_CYCLE_FIXEDPOINT;    
    }

  return nSineTable[ulLfoPhaseAccumulator >> 20] * in * 0.00001;
  }  

void audioHandler() {
  int i;
  uint32_t ulOutput[VOICENUM];  
  for (i = 0; i < VOICENUM; i++) {
    //ulPhaseAccumulator[i] += ulPhaseIncrement[i];   // 32 bit phase increment, see below
    ulPhaseAccumulator[i] += voiceFrequency[0] * INCREMENT_ONE_FIXEDPOINT;   // 32 bit phase increment, see below
    // if the phase accumulator over flows - we have been through one cycle at the current pitch,
    // now we need to reset the grains ready for our next cycle

    if(ulPhaseAccumulator[i] > SAMPLES_PER_CYCLE_FIXEDPOINT)
    {
      // DB 02/Jan/2012 - carry the remainder of the phase accumulator
      ulPhaseAccumulator[i] -= SAMPLES_PER_CYCLE_FIXEDPOINT; 
      //ulPhaseAccumulator[i] = 0;
    }
    
    // get the current sample  
    //uint32_t ulOutput = nSineTable[ulPhaseAccumulator[0]>>20] ^ nSquareTable[ulPhaseAccumulator[1]>>20]; 
    //uint32_t ulOutput = ((nSineTable[ulPhaseAccumulator[0]>>20] + nSquareTable[ulPhaseAccumulator[1]>>20]) >> 1) * envelopeVolume; 
    //uint32_t ulOutput = (nSineTable[ulPhaseAccumulator[0]>>20] * envelopeVolume[0]) ^ (nSquareTable[ulPhaseAccumulator[1]>>20] * envelopeVolume[1]) ^ (nSineTable[ulPhaseAccumulator[2]>>20] * envelopeVolume[2]) ^ (nSquareTable[ulPhaseAccumulator[3]>>20] * envelopeVolume[3]); 
    //uint32_t ulOutput = (nSineTable[ulPhaseAccumulator[0]>>20] * envelopeVolume[0]) ^ (nSquareTable[ulPhaseAccumulator[1]>>20] * 0) ^ (nSineTable[ulPhaseAccumulator[2]>>20] * 0) ^ (nSquareTable[ulPhaseAccumulator[3]>>20] * 0); 
    //ulOutput = ulOutput ^ (nSineTable[ulPhaseAccumulator[i]>>20] * envelopeVolume[i]);
    // filtered:
    //ulOutput = ulOutput + filter((nSineTable[ulPhaseAccumulator[i]>>20] * envelopeVolume[i]), q[i], f[i], fb[i]);
    // nonfiltered: 
    //ulOutput = ulOutput + (nSineTable[ulPhaseAccumulator[i]>>20] * envelopeVolume[i]);
    //ulOutput[i] = filter(*(&nSineTable[0] + (ulPhaseAccumulator[i]>>20)) * envelopeVolume[i], q[i], f[i], fb[i]); // no loud noise at start, when filter is here and not in for loop sampleOsc += ulOutput[i]
    //ulOutput[i] = *(&nSineTable[0] + (ulPhaseAccumulator[i]>>20)) * envelopeVolume[i] >> 10;
    ulOutput[i] = *(soundType[i] + (ulPhaseAccumulator[i]>>20)) * envelopeVolume[i] >> 10;
    //ulOutput[i] = *(&nSineTable[0] + (ulPhaseAccumulator[i]>>20));
    //globalOut = ulOutput[i];  // for debugging (Serial.println())
  }

  //ulOutput = ulOutput + filter((nSineTable[ulPhaseAccumulator[0]>>20] * envelopeVolume[0]), q[0], f[0]);
  //ulOutput = ulOutput + filter((nSquareTable[ulPhaseAccumulator[1]>>20] * envelopeVolume[1]), q[1], f[1]);    
  //ulOutput = ulOutput + filter((nSawTable[ulPhaseAccumulator[2]>>20] * envelopeVolume[2]), q[2], f[2]);  
  //ulOutput = ulOutput + filter((nTriangleTable[ulPhaseAccumulator[3]>>20] * envelopeVolume[3]), q[3], f[3]); 

  //ulOutput = ulOutput + (nSineTable[ulPhaseAccumulator[0]>>20] * envelopeVolume[0]);
  //ulOutput = ulOutput + (nSquareTable[ulPhaseAccumulator[1]>>20] * envelopeVolume[1]);    
  //ulOutput = ulOutput + (nSawTable[ulPhaseAccumulator[2]>>20] * envelopeVolume[2]);  
  //ulOutput = ulOutput + (nTriangleTable[ulPhaseAccumulator[3]>>20] * envelopeVolume[3]); 

  //ulOutput = lfo(ulOutput);

  int32_t sampleOsc;
  for (int i = 0; i < VOICENUM; i++)
  {
    sampleOsc += ulOutput[i];
  } 
  
  sampleOsc = sampleOsc / 4;

  
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
  //Serial.println("a");
}

void envelopeHandler() {
  //Serial.print(", envelopeVolume[3]: ");
  //Serial.println(envelopeVolume[0]);  
  int i;
  //Serial.println("progres");
  //Serial.print(", envelopeVolume[3]: ");
  //Serial.println(envelopeVolume[0]);
  for (i = 0; i < VOICENUM; i++) {
  //Serial.print(", envelopeVolume[3]: ");
  //Serial.println(envelopeVolume[0]);    
    switch (envelopeProgress[i]) {
      //Serial.print(", envelopeVolume[3]: ");
      //Serial.println(envelopeVolume[0]);      
      case 0: // ATTACK
        //Serial.println("case 0");
        //attackStartTime = millis();
        //Serial.println(millis());
        //Serial.println(attackStartTime);
        //Serial.println(millis() - attackStartTime);

        if ((millis() - attackStartTime[i]) > attackTime[i]) {
          decayStartTime[i] = millis();            
          envelopeProgress[i]= 1;
          }
        else {
          envelopeVolume[i] = map(millis(), attackStartTime[i], attackStartTime[i] + attackTime[i], 0, 1023);
          //Serial.print("i: ");          
          //Serial.print(i);
          //Serial.print(", envelopeVolume[3]: ");
          //Serial.println(envelopeVolume[0]);
          }
        break;
        
      case 1: // DECAY
        //Serial.println("case 1");
        if ((millis() - decayStartTime[i]) > decayTime[i]) {
          sustainStartTime[i] = millis();          
          envelopeProgress[i] = 2;
          }    
        else {
          envelopeVolume[i] = map(millis(), decayStartTime[i], decayStartTime[i] + decayTime[i], 1023, 768);
          //Serial.println(envelopeVolume);        
          }        
        break;

      case 2: // SUSTAIN
        //Serial.println("case 2");
        if ((millis() - sustainStartTime[i]) > sustainTime[i]) {
          releaseStartTime[i] = millis();          
          envelopeProgress[i] = 3;
          }    
        else {
          envelopeVolume[i] = sustainLevel[i];
          //Serial.println(envelopeVolume);        
        }
        break;

      case 3: // RELEASE
        //Serial.println("case 3");
        if ((millis() - releaseStartTime[i]) > releaseTime[i]) {
          envelopeProgress[i] = 255;
          }    
      else {
        envelopeVolume[i] = map(millis(), releaseStartTime[i], releaseStartTime[i] + releaseTime[i], 768, 0);
        //Serial.println(envelopeVolume[i]);        
        }
        break;

      case 255: // MUTE
        envelopeVolume[i] = 0;
        //Serial.println("case 255");
        break;       
      }
    }
  }

void trigger(int i) {
  //Serial.println("trigger");  
  attackStartTime[i] = millis();
  envelopeProgress[i] = 0;
  }

int st = 0;
void sequencer() {
  int i, j;

  //Serial.print("st: ");
  //Serial.println(st);
  
  for (j = 0; j < VOICENUM; j++) {
    if (sequences[j][st] == true)
      trigger(j);
      //Serial.print("j: ");
      //Serial.print(j);
      //Serial.print(", seq: ");
      //Serial.println(sequences[j][st]);
    }
    
  if (st == 7) 
    st = 0;
  else
    st++;
  }

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
  //Serial.println("---");  
  //Serial.println(justpressed[0]);    
  //Serial.println("---");  
}

void ledsHandler() {
  //Serial.println(voiceN);
  for (int i = 0; i < N; i++) {
    //digitalWrite(i + 30, seqLedState[i]);
    //Serial.print(sequences[voiceN][i]);
    digitalWrite(i + 30, sequences[voiceN][i]);
    }
    //Serial.println();    
  }

void lcdHandler() {
  lcd.print("ctrl: ");
  lcd.print(ctrl);  
  lcd.print("val: ");  
  lcd.print(val);  
  }

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
  if (voiceParameterN > (6 - 1))
    voiceParameterN = 0;
  }

void downVoiceParameter() {
  --voiceParameterN;  
  if (voiceParameterN < 0)
    voiceParameterN = (6 - 1);
  }
  
void setup() {
  Serial.begin(9600);  
  //Serial.println(SAMPLES_PER_CYCLE_FIXEDPOINT);      
  
  lcd.begin(16, 2);
  lcd.print("ArduinoSynth");  
  lcd.setCursor(0, 1);
  lcd.print("v 0.1");
  lcd.setCursor(14, 1);
  for (int i = 0; i < 5; i++) {
    lcd.print(5 - i);
    lcd.print("s");
    //delay(1000);
  }

  // buttons
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
  for (int i = 0; i < N; i++) {
    pinMode(i + 30, OUTPUT);
  }  
    
  createSineTable();
  createSquareTable(100);  
  createSawTable();
  createTriangleTable();
  //Serial.println(envelopeVolume[0]);  

  analogWrite(DAC0, 0);
  analogWrite(DAC1, 0);

  playSound();
  //Serial.println(SAMPLES_PER_CYCLE_FIXEDPOINT);    
}

int cnt = 0;

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.print("vol: ");
  //Serial.println(envelopeVolume[0]);
  //Serial.print("prog: ");
  //Serial.println(envelopeProgress[3]);    
  envelopeHandler();
  //Serial.println("start");
  buttonsHandler();
  ledsHandler();

  voiceParam[voiceParameterN][voiceN] = analogRead(0);

  // temp sequencer buttons
  for (int i = 0; i < (NUMBUTTONS - 2); i++) {   
    if (pressed[i] && sequences[voiceN][i]) {         // justpressed works equally bad here
      sequences[voiceN][i] = false;
      seqLedState[i] = LOW;
    }
    else if (pressed[i] && !sequences[voiceN][i]) {   // justpressed works equally bad here
      sequences[voiceN][i] = true;
      seqLedState[i] = HIGH;      
    }
  }

  // temp up down voice parameter
  if (justpressed[8]) // or justpressed? both work
    upVoiceParameter();  
  if (justpressed[9]) // or justpressed? both work
    downVoiceParameter();

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
  
  //Serial.println("---");  
  //Serial.println(justpressed[0]);  
  //Serial.println("---");
  //Serial.println(cnt);
  //Serial.println("end");
  //Serial.println(voiceParameterN);    
  //Serial.println(voiceParam[voiceParameterN][voiceN]);      
}
