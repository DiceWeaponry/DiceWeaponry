#include <PololuLedStrip.h>

#include "Volume3.h"
#include <MPU6050_tockn.h>

//#include <LowPower.h>

PololuLedStrip<2> ledStrip;


#define LED_COUNT 20
#define BUTTON_TIMEOUT 300
#define ACC_THRESHOLD 1.5
#define HIT_TIMEOUT 5000
#define QUEUE_TIMEOUT 20
#define SWING_TIMEOUT 150
#define HIT_THRESHHOLD 1.9
rgb_color colors[LED_COUNT];
MPU6050 tilt(Wire);



//////////////////////////////////////////////////////////////////////////////
// VARIABLES THAT DONT CHANGE
//////////////////////////////////////////////////////////////////////////////

const short speaker_pin = 9;
const short button1 = 6;
const short button2 = 5;
const short num_di_states = 6;
short selector = num_di_states - 1;
const short di_states[num_di_states] = { 4, 6, 8, 10, 12, 20 };
short button1_states[2] = { 1, 1 };
short button2_states[2] = { 1, 1 };

float ACC, GYR, COMPL, PREV_ACC = 0;
float gyroX, gyroY, gyroZ, accelX, accelY, accelZ, angleX, angleY, angleZ;
float k = 0.2;
long mpu_timer, roll_timer, hit_timer, queue_timer, swing_timer;

float accel_change[10] = {};
int curr_queue = 0;

float curr_hit_threshhold = HIT_THRESHHOLD;

long chromatic_timer;
bool chromatic_trigger = false;
bool roll_showing = false;
bool useVol = true;  // Are we using volume envelopes
bool lights_are_on = false,
     hit_trigger = false,
     swing_trigger = false,
     button_state = false,
     ready_to_roll = false,
     states_changed = false,
     volume_mode = false;

short current_volume = 1023;
const short num_vol_states = 5,
            max_vol = 1023,
            vol_increment = max_vol / num_vol_states,
            light_vol_multiplier = LED_COUNT / num_vol_states;

short curr_delay;
short curr_note_index = 0;
short start_note;
short random_roll;

short led_timeout = 20;
bool led_trigger = false;
long led_timer;

short roll_timeout;
bool roll_trigger = false;

bool button_trigger = false;
bool button_trigger2 = false;
unsigned long button_time;
long button_hold_time;

const short max_notes = 41;

enum Freq {
  a2 = 110,
  as2 = 117,
  b2 = 123,
  c3 = 131,
  cs3 = 139,
  d3 = 147,
  ds3 = 156,
  e3 = 165,
  f3 = 175,
  fs3 = 185,
  g3 = 196,
  gs3 = 208,
  a3 = 220,
  as3 = 233,
  b3 = 247,
  c4 = 262,
  cs4 = 277,
  d4 = 294,
  ds4 = 311,
  e4 = 330,
  f4 = 349,
  fs4 = 370,
  g4 = 392,
  gs4 = 415,
  a4 = 440,
  as4 = 466,
  b4 = 494,
  c5 = 523,
  cs5 = 554,
  d5 = 587,
  ds5 = 622,
  e5 = 659,
  f5 = 698,
  fs5 = 740,
  g5 = 784,
  gs5 = 831,
  a5 = 880,
  as5 = 932,
  b5 = 988,
  c6 = 1047,
  rest = 0
};

static short notes[max_notes] = {
  a2, as2, b2, c3, cs3, d3, ds3, e3, f3, fs3,
  g3, gs3, a3, as3, b3, c4, cs4, d4, ds4, e4,
  f4, fs4, g4, gs4, a4, as4, b4, c5, cs5, d5,
  ds5, e5, f5, fs5, g5, gs5, a5, as5, b5, c6,
  rest
};

byte lastBits = 0x00;
byte lBuf = 0x00;
byte rBuf = 0x00;


//////////////////////////////////////////////////////////////////////////////
// VARIABLES YOU CAN CHANGE
//////////////////////////////////////////////////////////////////////////////


char double_button_timeout = 500;
unsigned long double_button_timer;
int diResults = 1;
float swing_threshhold = 0.7;
float initialSpeed = 5.0;
const float ease_factor = 1.5;
const short max_delay = 150;
short chromatic_timeout = 10;
const short beats_per_minute = 180;
const short beats_per_second = 60000 / beats_per_minute;
const short button_hold_threshhold = 250;

const short di_state_colors[LED_COUNT] = { 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230 };
short _red[3] = { 50, 0, 0 };
short _green[3] = { 0, 10, 0 };
short _blue[3] = { 0, 0, 10 };
short _purple[3] = { 10, 0, 0 };
short _white[3] = { 255, 255, 255 };

static const short myMelodyLength = 21;
static const short myMelody[myMelodyLength] = { 0, 2, 3, 2, 3, 5, 3, 5, 7, 5, 7, 8, 7, 8, 10, 12, 24, 12, 24, 12, 24 };
static const short myDuration[myMelodyLength] = { 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 1, 8, 8, 8, 8, 1 };

static const short failLength = 2;
static const short failMelody[failLength] = { 13, 0 };
static const short failMelodyDuration[failLength] = { 0, 0 };

static short oneFourFive[] = { 0, 4, 5, 12 };

static const byte startupLength = 3;
static short startupMelody[startupLength] = { 12, 0, 24 };
//static bool interruptFriendly = true;


//////////////////////////////////////////////////////////////////////////////
// SETUP
//////////////////////////////////////////////////////////////////////////////


void setup() {
  //PololuLedStripBase::interruptFriendly = true;
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(speaker_pin, OUTPUT);
  Serial.begin(9600);
  Serial.println("setup");

  Wire.begin();
  tilt.begin();
  if (digitalRead(button1) == LOW || digitalRead(button2) == LOW) current_volume = 255;
  //playMelody(myMelody, myDuration, myMelodyLength, beats_per_second);
  powerOn();
}



//////////////////////////////////////////////////////////////////////////////
// HELPERS
//////////////////////////////////////////////////////////////////////////////

short getIndex(short freq, short arr[]) {
  for (short i = 0; i < max_notes; i++) {
    if (arr[i] == freq) return i;
  }
  return 0;
}
// void printArr(float arr[]) {
//   for (int i = 0; i < 10; i++) {
//     Serial.print(arr[i]);
//     Serial.print(" ");
//   }
//   Serial.println();
// }

float minMaxDiff(float arr[]) {
  float _min = arr[0];
  float _max = arr[0];
  for (uint16_t i = 0; i < 10; i++) {
    if (arr[i] < _min) _min = arr[i];
    if (arr[i] > _max) _max = arr[i];
  }
  return _max - _min;
}

float vectorAvg(float x, float y, float z) {
  float v = sq(x) + sq(y) + sq(z);
  v = sqrt(v);
  return v;
}

void addAccToQueue() {
  if ((millis() - queue_timer) > QUEUE_TIMEOUT) {
    queue_timer = millis();
    tilt.update();
    ACC = vectorAvg(tilt.getAccX(), tilt.getAccY(), tilt.getAccZ());
    if (curr_queue > 9) curr_queue = 0;
    accel_change[curr_queue] = ACC;
    curr_queue++;
  }
}

void swapshort(short *a, short *b) {
  short c = *a;
  *a = *b;
  *b = c;
}




//////////////////////////////////////////////////////////////////////////////
// RANDOM GENERATOR
//////////////////////////////////////////////////////////////////////////////

//Based on the work of B. Peng
//https://gist.github.com/bloc97/b55f684d17edd8f50df8e918cbc00f94

byte rotate(byte b, byte n) {
  return (b << n) | (b >> (8 - n));
}

byte trueRotateRandom() {
  byte bits = 0;
  byte leftSample, rightSample;
  byte leftBit, rightBit;
  byte lastBuf = lBuf ^ rBuf;
  for (byte i = 0; i < 4; i++) {
    delayMicroseconds(random(10));
    leftSample = analogRead(A0) & 0x3f;
    delayMicroseconds(random(10));
    rightSample = analogRead(A0) & 0x3f;
    bits = bits ^ rotate(leftSample, i);
    bits = bits ^ rotate(rightSample, 7 - i);
    for (byte j = 0; j < 8; j++) {
      leftBit = (leftSample >> j) & 1;
      rightBit = (leftSample >> j) & 1;
      if (leftBit != rightBit) {
        if (lastBuf % 2 == 0) lBuf = bitWrite(lBuf, 0, leftBit);
      } else {
        rBuf = bitWrite(lBuf, 7, leftBit);
      }
    }
  }
  lastBits = lastBits ^ (lastBits >> 3) ^ (lastBits << 5) ^ (lastBits >> 4);  //One round of XORShift PRNG algorithm for statistical stability
  lastBits = lastBits ^ bits;                                                 //Reseed the PRNG with TRNG values
  return lastBits ^ lBuf ^ rBuf;
}



uint8_t randomByteFromAccel() {
  tilt.update();
  const uint8_t minEntropyScale = 4;  //Resolution of the MPU sensor, the one I used outputed integers that are multiples of 4.
  int16_t xBits = tilt.getRawAccX();
  int16_t yBits = tilt.getRawAccY();
  int16_t zBits = tilt.getRawAccZ();

  uint8_t real4xBits = (xBits / minEntropyScale) & 0xF;
  uint8_t real4yBits = (yBits / minEntropyScale) & 0xF;
  uint8_t real4zBits = (zBits / minEntropyScale) & 0xF;

  uint8_t random8Bits = ((real4xBits & 0x3) << 6) ^ (real4zBits << 4) ^ (real4yBits << 2) ^ real4xBits ^ (real4zBits >> 2);

  return random8Bits;
}

//////////////////////////////////////////////////////////////////////////////
// VOLUME
//////////////////////////////////////////////////////////////////////////////

void volumeMode() {
  //  if (millis() - button_time > BUTTON_TIMEOUT) {
  //    if (doubleButtonPress()) {
  //      volume_mode = !volume_mode;
  //      button_time = millis();
  //      updateLights(true, _blue, 0, di_states[selector]);
  //    } else {
  //      byte prev_volume = current_volume;
  //      if (button1_states[0] == 1 && button1_states[1] == 0) {
  //        button_time = millis();
  //        if (current_volume + vol_increment > max_vol) current_volume = 0;
  //        else current_volume = current_volume + vol_increment;
  //        allLightsOff();
  //        lightVolumeFade(150, prev_volume, 0, (light_vol_multiplier * current_volume / vol_increment) - 1);
  //      } else if (button2_states[0] == 1 && button2_states[1] == 0) {
  //        button_time = millis();
  //        if (current_volume - vol_increment < 0) current_volume = max_vol;
  //        else current_volume = current_volume - vol_increment;
  //        allLightsOff();
  //        lightVolumeFade(150, prev_volume, (light_vol_multiplier * current_volume / vol_increment) - 1, 0);
  //      }
  //    }
  //  }
}


//////////////////////////////////////////////////////////////////////////////
// ACCELEROMETER TRIGGERS
//////////////////////////////////////////////////////////////////////////////

void swing() {
  int exxx = (minMaxDiff(accel_change) * 333);
  exxx = constrain(exxx, 250, 1047);
  //Serial.print("exxx = ");
  //Serial.println(exxx);

  if ((millis() - swing_timer) > SWING_TIMEOUT) {


    if (minMaxDiff(accel_change) > swing_threshhold) {
      if (roll_trigger) rollResults();
      else {
       

          for (uint16_t volume = 0; volume < 1000; volume ++) {
            //setLight(volume / 50, 224, 50);
             
            addAccToQueue();
            vol.tone(speaker_pin, exxx, volume);
            // if (roll_showing == false) {
            // setLight(volume / 50, 224, 50);
            //  }
            delayMicroseconds(20);
          }
          for (uint16_t volume = 1000; volume > 0; volume --) {
            
            addAccToQueue();
            vol.tone(speaker_pin, exxx, volume);
            // if (roll_showing == false) {
            // setLight(volume / 50, 240, 4);
            //  }
            delayMicroseconds(20);
          }
        
        //delay(500);
      }
    }
    swing_timer = millis();
  }
}

//////////////////////////////////////////////////////////////////////////////
// AUDIO
//////////////////////////////////////////////////////////////////////////////

//Bends pitch from first to second.
void pitchBend(short first, short last) {
  if (first < last) {
    for (short i = first; i < last; i++) {
      vol.tone(speaker_pin, i, current_volume);
      delay(5);
    }
  } else {
    for (short i = first; i >= last; i--) {
      vol.tone(speaker_pin, i, current_volume);
      delay(5);
    }
  }
  vol.noTone();
}


//Does a volume bend on the note from the first to the last
void volumeBend(short note, short first, short last) {
  if (first < last) {
    for (short i = first; i < last; i++) {
      vol.tone(speaker_pin, note, i);
      delay(5);
    }
  } else {
    for (short i = first; i >= last; i--) {
      vol.tone(speaker_pin, note, i);
      delay(5);
    }
  }
  vol.noTone();
}


//////////////////////////////////////////////////////////////////////////////
// LIGHTS
//////////////////////////////////////////////////////////////////////////////


//Fades the hue from firstLight to lastLight
//Edit "rate_of_change" to quicken or slow the speed of the lights
void lightFade(uint16_t hue, short firstLight, short lastLight, short starting_brightness, short ending_brightness, short rate_of_change) {
  if (firstLight >= 0 && lastLight >= 0) {
    if (starting_brightness <= ending_brightness) {
      for (short i = starting_brightness; i < ending_brightness; i = i + rate_of_change) {
        for (short j = firstLight; j <= lastLight; j++)
          colors[j] = hsvToRgb(hue, 255, i);
        if (singleButtonPress()) {
          vol.noTone();
          return;
        }

        ledStrip.write(colors, LED_COUNT);
      }
    } else {
      for (short i = ending_brightness; i >= ending_brightness; i = i - rate_of_change) {
        for (short j = firstLight; j <= lastLight; j++)
          colors[j] = hsvToRgb(hue, 255, i);
        if (singleButtonPress()) {
          vol.noTone();
          return;
        }
        ledStrip.write(colors, LED_COUNT);
      }
    }
  }
}

// Converts a color from HSV to RGB.
// h is hue, as a number between 0 and 360.
// s is the saturation, as a number between 0 and 255.
// v is the value, as a number between 0 and 255.
rgb_color hsvToRgb(uint16_t h, uint8_t s, uint8_t v) {
  uint8_t f = (h % 60) * 255 / 60;
  uint8_t p = (255 - s) * (uint16_t)v / 255;
  uint8_t q = (255 - f * (uint16_t)s / 255) * (uint16_t)v / 255;
  uint8_t t = (255 - (255 - f) * (uint16_t)s / 255) * (uint16_t)v / 255;
  uint8_t r = 0, g = 0, b = 0;
  switch ((h / 60) % 6) {
    case 0:
      r = v;
      g = t;
      b = p;
      break;
    case 1:
      r = q;
      g = v;
      b = p;
      break;
    case 2:
      r = p;
      g = v;
      b = t;
      break;
    case 3:
      r = p;
      g = q;
      b = v;
      break;
    case 4:
      r = t;
      g = p;
      b = v;
      break;
    case 5:
      r = v;
      g = p;
      b = q;
      break;
  }
  return rgb_color(r, g, b);
}

//Turns all lights off
void allLightsOff() {
  for (short i = 0; i < LED_COUNT; i++) {
    colors[i] = rgb_color(0, 0, 0);
  }
  ledStrip.write(colors, LED_COUNT);
}

//Sets all lights to supplied RGB color
void updateLights(bool clearLights, short rgb[3], short first, short last) {
  if (clearLights) allLightsOff();
  for (int i = first; i < last; i++) {
    colors[i] = rgb_color(rgb[0], rgb[1], rgb[2]);
  }
  ledStrip.write(colors, LED_COUNT);
}

//given a starting point, bursts outward
void burstFromPoint(short rgb[3], char startingPoint) {

  char head = startingPoint;
  char tail = startingPoint;
  char currColor = 0;

  //initiateFirstColor
  colors[startingPoint] = rgb_color(rgb[0], rgb[1], rgb[2]);
  ;
  ledStrip.write(colors, LED_COUNT);

  //burst from starting point outward in rainbow pattern until end of lights is reached
  for (int i = 0; i < LED_COUNT; ++i) {
    //reset the color counter
    if (startingPoint + i > LED_COUNT || startingPoint - i <= 0) break;
    //advance the tail LED
    colors[startingPoint - i] = rgb_color(rgb[0], rgb[1], rgb[2]);
    //advance the head LED
    colors[startingPoint + i] = rgb_color(rgb[0], rgb[1], rgb[2]);
    //fadeColors();
    ledStrip.write(colors, LED_COUNT);
    delay(50);
  }
  allLightsOff();
}

void setLight(short lightNum, short hue, short brightness) {
  colors[lightNum] = hsvToRgb(hue, 255, brightness);
  ledStrip.write(colors, LED_COUNT);
}



//////////////////////////////////////////////////////////////////////////////
// AUDIO/LIGHTS
//////////////////////////////////////////////////////////////////////////////

//Fades the hue from firstLight to lastLight up to the current volume (max 255)
void lightVolumeFade(uint16_t hue, byte prev_volume, short firstLight, short lastLight) {
  if (firstLight >= 0 && lastLight >= 0) {
    if (firstLight <= lastLight) {
      for (short i = prev_volume; i < current_volume; i = i + 5) {
        for (short j = firstLight; j <= lastLight; j++)
          colors[j] = hsvToRgb(hue, 255, i);
        if (singleButtonPress()) {
          vol.noTone();
          return;
        }
        ledStrip.write(colors, LED_COUNT);
        vol.tone(speaker_pin, notes[lastLight], i);
      }
    } else {
      if (prev_volume < current_volume) {
        lightVolumeFade(hue, 200, lastLight, firstLight);
        return;
      }
      for (short i = prev_volume; i >= current_volume; i = i - 5) {
        for (short j = lastLight; j <= firstLight; j++)
          colors[j] = hsvToRgb(hue, 255, i);
        if (singleButtonPress()) {
          vol.noTone();
          return;
        }
        ledStrip.write(colors, LED_COUNT);
        vol.tone(speaker_pin, notes[firstLight], i);
      }
    }
    vol.noTone();
  }
}

//Supply the melody, duration, size of melody & duration arrays, beats per second (60000/bpm)
void playMelody(short melody[], short duration[], short numNotes, short bps) {
  allLightsOff();
  short lightNum = 0;
  for (short i = 0; i < numNotes; i++) {
    if (myMelody[i] >= LED_COUNT) lightNum = LED_COUNT - 1;
    else lightNum = melody[i];
    colors[lightNum] = rgb_color(random(255), random(255), random(255));
    ledStrip.write(colors, LED_COUNT);
    vol.tone(speaker_pin, notes[melody[i]], current_volume);
    delay(bps / duration[i]);
    colors[lightNum] = rgb_color(0, 0, 0);
    ledStrip.write(colors, LED_COUNT);
  }
  vol.noTone();
}

//Goes up the chromatic scale starting with supplied note, up to supplied note + LED_COUNT
void chromaticDown(short firstNote, short startingLight) {
   roll_showing = true;
  allLightsOff();
  for (short i = startingLight; i > 0; i--) {
    addAccToQueue();
    colors[(i - 1)] = rgb_color(0, 0, 10);
    ledStrip.write(colors, LED_COUNT);
    vol.tone(speaker_pin, notes[firstNote + (i - 1)], current_volume);
    delay(chromatic_timeout);
  }
  vol.noTone();
}

//Goes up the chromatic scale starting with supplied note, up to supplied note + LED_COUNT
void chromaticUp(short firstNote, short lastLight) {
  roll_showing = false;
  allLightsOff();
  for (short i = 0; i <= lastLight; i++) {
    addAccToQueue();
    colors[i] = rgb_color(0, 0, 10);
    ledStrip.write(colors, LED_COUNT);
    vol.tone(speaker_pin, notes[firstNote + i], current_volume);
    delay(chromatic_timeout);
  }
  vol.noTone();
}

void failRoll(short failRoll) {
  short bps = 60000 / 100;
  updateLights(true, _red, 0, di_states[selector]);
  vol.tone(speaker_pin, notes[di_states[selector]], current_volume);
  delay(bps + 5);
  updateLights(true, _red, 0, failRoll);
  vol.tone(speaker_pin, notes[failRoll], current_volume);
  delay(bps + 5);
  vol.noTone();
}

void critRoll(short critRoll) {
  chromatic_timeout = 200 / critRoll;
  chromaticUp(0, critRoll);
  chromaticUp(0, critRoll);
  allLightsOff();
  lightVolumeFade(255, 0, 0, critRoll);
  chromatic_timeout = 10;
  gradLoop();
}

//Randomly lights up the LED strip
//Supply the rolltime in milliseconds
void startupRandLightFlash(unsigned long roll_time) {
  unsigned long currTime = millis();
  short currNote = 0;
  led_timeout = 30;
  while (millis() - currTime < roll_time) {
    vol.tone(speaker_pin, notes[startupMelody[currNote]], current_volume);
    delay(roll_time / 6);
    vol.noTone();
    currNote++;
    for (short i = 0; i < 5; i++) {
      randomSeed(analogRead(0));
      vol.tone(speaker_pin, random(500), current_volume);
      short led = random(LED_COUNT);
      short r = random(255);
      short g = random(255);
      short b = random(255);
      colors[led] = rgb_color(r, g, b);
      ledStrip.write(colors, LED_COUNT);
      delay(roll_time / 25);
      colors[led] = rgb_color(0, 0, 0);
      ledStrip.write(colors, LED_COUNT);
      delay(roll_time / 25);
    }
  }
}


void startupBurst() {
  char i = 0;
  vol.tone(speaker_pin, notes[startupMelody[i++]], current_volume);
  burstFromPoint(_red, LED_COUNT - 5);
  vol.noTone();
  ;
  burstFromPoint(_green, 5);
  vol.noTone();
  vol.tone(speaker_pin, notes[startupMelody[i++]], current_volume);
  burstFromPoint(_blue, LED_COUNT / 2);
  vol.noTone();
}

//Plays the startup random flash and then does a chromatic
void powerOn() {
  //startupBurst();
  startupRandLightFlash(1000);
  allLightsOff();
  chromaticUp(16, LED_COUNT);
}

//Using global variable "selector", play corresponding note and ease into lights up to di_states[selector]
void select() {
  Serial.print("Select = ");
  Serial.println(di_states[selector]);
  allLightsOff();
  byte count = 0;
  unsigned long time_at_last_action = millis();
  unsigned short curr_delay = max_delay;
  vol.tone(speaker_pin, notes[(start_note + oneFourFive[curr_note_index])], current_volume);
  while (millis() - time_at_last_action < curr_delay) {
    if (singleButtonPress()) {
      updateLights(true, _blue, 0, di_states[selector]);

      vol.noTone();
      return;
    }
  }
  vol.noTone();
  while (count < di_states[selector] || count == 0) {
    if (singleButtonPress()) {
      updateLights(true, _blue, 0, di_states[selector]);
      buttonPress();
      break;
    }
    if (millis() - time_at_last_action >= curr_delay || count == 0) {
      addAccToQueue();
      curr_delay = max_delay / (pow(ease_factor, di_states[selector] - count));
      colors[count] = rgb_color(0, 0, 10);
      ledStrip.write(colors, LED_COUNT);
      count++;
      time_at_last_action = millis();
    }
  }
}


//Eases into numLights from zero.
//clearLights - Do you want to clear the lights, yes or no?
//numLights - How many lights do you want to light up?
//rgb - What color? (In format short rgb[3] = {255,255,255})
void easeLights(bool clearLights, short numLights, short rgb[3]) {

  if (clearLights) allLightsOff();
  byte count = 0;
  unsigned long time_at_last_action = 0;
  unsigned short curr_delay = 0;
  while (count < numLights) {
    if (singleButtonPress()) {
      updateLights(true, _blue, 0, di_states[selector]);
      buttonPress();
      break;
    }
    if (millis() - time_at_last_action >= curr_delay || count == 0) {
      vol.noTone();
      addAccToQueue();
      curr_delay = max_delay / (pow(ease_factor, numLights - count));
      if (curr_delay < 30) curr_delay = 30;
      colors[count] = rgb_color(rgb[0], rgb[1], rgb[2]);
      ledStrip.write(colors, LED_COUNT);
      vol.tone(speaker_pin, notes[start_note + count], current_volume);
      count++;
      time_at_last_action = millis();
    }
  }
  vol.noTone();
}



void readyLights(bool clearLights, short numLights, short rgb[3]) {



  if (clearLights) allLightsOff();

  byte count = 0;
  unsigned long time_at_last_action = 0;
  unsigned short curr_delay = 0;
  while (count < numLights) {
    //    digitalRead
    if (millis() - time_at_last_action >= curr_delay || count == 0) {
      vol.noTone();
      addAccToQueue();
      curr_delay = max_delay / (pow(ease_factor, numLights - count));
      if (curr_delay < 30) curr_delay = 30;
      colors[count] = rgb_color(rgb[0], rgb[1], rgb[2]);
      ledStrip.write(colors, LED_COUNT);
      vol.tone(speaker_pin, notes[start_note + count], current_volume);

      count++;
      time_at_last_action = millis();
    }
  }

  vol.noTone();
}
//Selects random light, triggers chromatic down and triggers lights/audio up to that random number
void rollResults() {
  //byte diRoll = 1 + ((di_states[selector]) * trueRotateRandom() / 255);
  byte diRoll = ((di_states[selector]) * trueRotateRandom() / 255);

  if (diRoll <= 0) {
    diRoll = 1;
  } else {
  }
  diResults = diRoll;
  chromaticDown(16, di_states[selector]);
  Serial.print("results = ");
  Serial.println(diRoll);
 

  if (diRoll == 1) {
    swing_threshhold = 0.7;
    //Serial.println(swing_threshhold);
    failRoll(diRoll);
  } else if (diRoll == 2) {
    swing_threshhold = 0.7;
    //Serial.println(swing_threshhold);
    easeLights(true, 1, _purple);
    setLight(1, 100, 100);
    uint16_t v = 1000;
    while (v > 0) {
      //vol.tone(speaker_pin, 2090, v);
      vol.tone(speaker_pin, diRoll * 110, v);  // ting!
      delay(10);
      v -= 10;
    }
  } else if (diRoll == 3) {
    swing_threshhold = 0.7;
    //Serial.println(swing_threshhold);
    easeLights(true, 2, _purple);
    setLight(2, 100, 100);
    int16_t v = 1000;
    while (v > 0) {
      //vol.tone(speaker_pin, 2090, v);
      vol.tone(speaker_pin, diRoll * 110, v);  // ting!
      delay(10);
      v -= 10;
    }
  } else if (diRoll == 4) {
    swing_threshhold = 0.7;
    //Serial.println(swing_threshhold);
    easeLights(true, 3, _purple);
    setLight(3, 100, 100);
    int16_t v = 1000;
    while (v > 0) {
      //vol.tone(speaker_pin, 2090, v);
      vol.tone(speaker_pin, diRoll * 110, v);  // ting!
      delay(10);
      v -= 10;
    }
  } else if (diRoll == 5) {
    swing_threshhold = 0.7;
    //Serial.println(swing_threshhold);
    easeLights(true, 4, _purple);
    setLight(4, 100, 100);
    int16_t v = 1000;
    while (v > 0) {
      //vol.tone(speaker_pin, 2090, v);
      vol.tone(speaker_pin, diRoll * 110, v);  // ting!
      delay(10);
      v -= 10;
    }
  } else if (diRoll == 6) {
    swing_threshhold = 0.7;
    //Serial.println(swing_threshhold);
    easeLights(true, 5, _purple);
    setLight(5, 100, 100);
    int16_t v = 1000;
    while (v > 0) {
      //vol.tone(speaker_pin, 2090, v);
      vol.tone(speaker_pin, diRoll * 110, v);  // ting!
      delay(10);
      v -= 10;
    }
  } else if (diRoll == 7) {
    swing_threshhold = 0.7;
    //Serial.println(swing_threshhold);
    easeLights(true, 6, _purple);
    setLight(6, 100, 100);
    int16_t v = 1000;
    while (v > 0) {
      //vol.tone(speaker_pin, 2090, v);
      vol.tone(speaker_pin, diRoll * 110, v);  // ting!
      delay(10);
      v -= 10;
    }
  } else if (diRoll == 8) {
    swing_threshhold = 0.7;
    //Serial.println(swing_threshhold);
    easeLights(true, 7, _purple);
    setLight(7, 100, 100);
    int16_t v = 1000;
    while (v > 0) {
      //vol.tone(speaker_pin, 2090, v);
      vol.tone(speaker_pin, diRoll * 110, v);  // ting!
      delay(10);
      v -= 10;
    }
  } else if (diRoll == 9) {
    swing_threshhold = 0.7;
    //Serial.println(swing_threshhold);
    easeLights(true, 8, _purple);
    setLight(8, 100, 100);
    int16_t v = 1000;
    while (v > 0) {
      //vol.tone(speaker_pin, 2090, v);
      vol.tone(speaker_pin, diRoll * 110, v);  // ting!
      delay(10);
      v -= 10;
    }
  } else if (diRoll == 10) {
    swing_threshhold = 0.7;
    //Serial.println(swing_threshhold);
    easeLights(true, 9, _purple);
    setLight(9, 100, 100);
    int16_t v = 1000;
    while (v > 0) {
      //vol.tone(speaker_pin, 2090, v);
      vol.tone(speaker_pin, diRoll * 110, v);  // ting!
      delay(10);
      v -= 10;
    }
  } else if (diRoll == 11) {
    swing_threshhold = 0.7;
    //Serial.println(swing_threshhold);
    easeLights(true, 10, _purple);
    setLight(10, 100, 100);
    int16_t v = 1000;
    while (v > 0) {
      //vol.tone(speaker_pin, 2090, v);
      vol.tone(speaker_pin, diRoll * 110, v);  // ting!
      delay(10);
      v -= 10;
    }
  } else if (diRoll == 12) {
    swing_threshhold = 0.7;
    //Serial.println(swing_threshhold);
    easeLights(true, 11, _purple);
    setLight(11, 100, 100);
    int16_t v = 1000;
    while (v > 0) {
      //vol.tone(speaker_pin, 2090, v);
      vol.tone(speaker_pin, diRoll * 110, v);  // ting!
      delay(10);
      v -= 10;
    }
  } else if (diRoll == 13) {
    swing_threshhold = 0.7;
    //Serial.println(swing_threshhold);
    easeLights(true, 12, _purple);
    setLight(12, 100, 100);
    int16_t v = 1000;
    while (v > 0) {
      //vol.tone(speaker_pin, 2090, v);
      vol.tone(speaker_pin, diRoll * 110, v);  // ting!
      delay(10);
      v -= 10;
    }
  } else if (diRoll == 14) {
    swing_threshhold = 0.7;
    //Serial.println(swing_threshhold);
    easeLights(true, 13, _purple);
    setLight(13, 100, 100);
    int16_t v = 1000;
    while (v > 0) {
      //vol.tone(speaker_pin, 2090, v);
      vol.tone(speaker_pin, diRoll * 110, v);  // ting!
      delay(10);
      v -= 10;
    }
  } else if (diRoll == 15) {
    swing_threshhold = 0.7;
    //Serial.println(swing_threshhold);
    easeLights(true, 14, _purple);
    setLight(14, 100, 100);
    int16_t v = 1000;
    while (v > 0) {
      //vol.tone(speaker_pin, 2090, v);
      vol.tone(speaker_pin, diRoll * 110, v);  // ting!
      delay(10);
      v -= 10;
    }
  } else if (diRoll == 16) {
    swing_threshhold = 0.7;
    //Serial.println(swing_threshhold);
    easeLights(true, 15, _purple);
    setLight(15, 100, 100);
    int16_t v = 1000;
    while (v > 0) {
      //vol.tone(speaker_pin, 2090, v);
      vol.tone(speaker_pin, diRoll * 110, v);  // ting!
      delay(10);
      v -= 10;
    }
  } else if (diRoll == 17) {
    swing_threshhold = 0.7;
    //Serial.println(swing_threshhold);
    easeLights(true, 16, _purple);
    setLight(16, 100, 100);
    int16_t v = 1000;
    while (v > 0) {
      //vol.tone(speaker_pin, 2090, v);
      vol.tone(speaker_pin, diRoll * 110, v);  // ting!
      delay(10);
      v -= 10;
    }
  } else if (diRoll == 18) {
    swing_threshhold = 0.7;
    //Serial.println(swing_threshhold);
    easeLights(true, 17, _purple);
    setLight(17, 100, 100);
    int16_t v = 1000;
    while (v > 0) {
      //vol.tone(speaker_pin, 2090, v);
      vol.tone(speaker_pin, diRoll * 110, v);  // ting!
      delay(10);
      v -= 10;
    }
  } else if (diRoll == 19) {
    swing_threshhold = 0.7;
    //Serial.println(swing_threshhold);
    easeLights(true, 18, _purple);
    setLight(18, 100, 100);
    int16_t v = 1000;
    while (v > 0) {
      //vol.tone(speaker_pin, 2090, v);
      vol.tone(speaker_pin, diRoll * 110, v);  // ting!
      delay(10);
      v -= 10;
    }
  } else if (diRoll == 20) {
    easeLights(true, 20, _purple);

    critRoll(diRoll - 1);
  } else if (diRoll > 20) {
    easeLights(true, 20, _purple);

    critRoll(diRoll - 1);
  } else {
    easeLights(true, 20, _purple);

    critRoll(diRoll - 1);
  }

  //else if (diRoll == di_states[selector]) critRoll(diRoll - 1);



  // else {
  //   //allLightsOff();
  //   easeLights(true, diRoll - 1, _purple);
  //   //setLight(diRoll-1, 255/diRoll, 100);
  //   setLight(diRoll - 1, 100, 100);
  //   uint16_t v = 1000;
  //   while (v > 0) {
  //     //vol.tone(speaker_pin, 2090, v);
  //     vol.tone(speaker_pin, diRoll * 110, v);  // ting!
  //     delay(10);
  //     v -= 10;
  //   }
  //   //ching();


  //   // lightFade(0, diRoll, di_states[selector]-1, 50, 10, 5);

  //   //setLight(di_states[selector], 1, 255);
  // }
  ready_to_roll = false;
  roll_trigger = false;
}

//////////////////////////////////////////////////////////////////////////////
// BUTTONS
//////////////////////////////////////////////////////////////////////////////

//Updates array of both buttons
//  First element is the last state
//  Second element is the state before last
void updateButtonStates() {
  button1_states[1] = button1_states[0];
  button1_states[0] = digitalRead(button1);
  button2_states[1] = button2_states[0];
  button2_states[0] = digitalRead(button2);
}

//Checks if both buttons are pressed at the same time
bool doubleButtonPress() {
  if (digitalRead(button1) == LOW && digitalRead(button2) == LOW) return true;
  return false;
}

bool singleButtonPress() {
  updateButtonStates();
  if ((button1_states[0] == 1 && button1_states[1] == 0) || (button2_states[0] == 1 && button2_states[1] == 0)) return true;
  else return false;
}

//Checks to see if button one and button two are pressed.
//If button one is pressed, move the di selection up
//If button two is pressed, ready the roll.
void buttonPress() {



  if (millis() - button_time > BUTTON_TIMEOUT) {
    //if (doubleButtonPress()) {                                        //both buttons pressed
    // volume_mode = !volume_mode;
    // button_time = millis();
    // updateLights(true, _white, 0, light_vol_multiplier * current_volume / vol_increment);
    // } else {
    if ((button1_states[0] == 1 && button1_states[1] == 0)) {  //button one pressed
      Serial.println("butt 1");

      button_time = millis();
      curr_note_index = 0;
      ready_to_roll = false;
      roll_trigger = false;
      selector++;
      if (selector >= num_di_states) selector = 0;
      start_note = selector;
      select();
    } else if (button2_states[0] == 1 && button2_states[1] == 0) {  //button two pressed
      Serial.println("butt 2");
      
      button_time = millis();
      curr_note_index = 0;
      start_note = selector;
      if (roll_trigger) {

        swing_threshhold = 0.7;
        //Serial.println(swing_threshhold);
        roll_trigger = false;
        ready_to_roll = false;

        chromaticDown(16, di_states[selector]);

      } else {
        diResults = 1;
        swing_threshhold = 1.5;
        //Serial.println(swing_threshhold);
        readyLights(true, di_states[selector], { _green });
        // lightFade(0, 20, 0, 50, 50, 100);


        // for (uint16_t i = 0; i < 360; i++) {

        //   readyLights(false, 20, (i, 100, 255));
        // }

        // Write the colors to the LED strip.
        //ledStrip.write(colors, 20);

        delay(10);


        roll_trigger = true;
        ready_to_roll = true;
      }
    }
  }
}

void flickerLights_green() {

  int randomLight = random(LED_COUNT);
  int flickerIntensity = random(1, 10);
  colors[randomLight] = hsvToRgb(120, 255, flickerIntensity);
  ledStrip.write(colors, LED_COUNT);
  delay(random(5, 10));
}
void flickerLights_red() {

  int randomLight = random(diResults - 1);
  int flickerIntensity = random(1, 10);
  colors[randomLight] = hsvToRgb(1, 255, flickerIntensity);
  ledStrip.write(colors, diResults - 1);
  delay(random(5, 10));
}


void ching() {
  // vol.tone(speaker_pin, 1025, 1023);  // pa
  // delay(70);
  uint16_t ving = 1000;
  while (ving > 0) {
    //vol.tone(speaker_pin, 2090, v);
    vol.tone(speaker_pin, 2090, ving);  // ting!
    delay(10);
    ving -= 10;
  }
}
void randoblink() {
  //makes the onboard red led blink randomly
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(random(10, 100));           // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(random(10, 100));
}

void gradLoop() {
  //setLight(diResults, 100, 100);
 
  // red strobing effect on results
  byte time = millis() >> 3;
   //allLightsOff();
  //setLight(diResults-1, 100, 100);
  for (uint16_t i = 0; i < diResults - 1; i++) {
    byte x = time - 8 * i;
    colors[i] = hsvToRgb(0, 255, x / 10);
    
    //Serial.println(x / 5);
  }
  // if (roll_showing = true) {
  //   setLight(diResults-1, 100, 100);
  // }
  // Write the colors to the LED strip.
  ledStrip.write(colors, diResults - 1);

  delay(10);
}
void gradLoopCRIT() {
  // red strobing effect on results
  byte time = millis() >> 3;
  for (uint16_t i = 0; i < diResults - 1; i++) {
    byte x = time - 8 * i;
    colors[i] = hsvToRgb(0, 0, x / 5);
    //Serial.println(x / 5);
  }

  // Write the colors to the LED strip.
  //ledStrip.write(colors, diResults - 1);

  delay(10);
}

void rainbow() {
  // Update the colors.
  uint16_t time = millis() >> 2;
  for (uint16_t i = 0; i < 20; i++) {
    byte x = (time >> 2) - (i << 3);
    colors[i] = hsvToRgb((uint32_t)x * 359 / 256, 255, 15);
  }

  // Write the colors to the LED strip.
  ledStrip.write(colors, 20);

  delay(10);
}
// Function to animate the LEDs with adjustable speed
// void animateLEDs() {
//   // Move a single green light from LED 1 to LED 20 with easing
//   for (int i = 0; i < LED_COUNT; i++) {
//     // Turn off all LEDs
//     for (int j = 0; j < LED_COUNT; j++) {
//       colors[j] = rgb_color(0, 0, 0);
//     }
//     // Turn on the current LED to green
//     colors[i] = rgb_color(0, 255, 0);
//     ledStrip.write(colors, LED_COUNT);

//     // Calculate the delay for the current step
//     float stepDelay = initialSpeed * easeOut(i / (float)LED_COUNT);
//     delay(stepDelay);  // Apply the calculated delay
//   }

//   // Ease into the end of the animation by gradually reducing the brightness
//   for (int brightness = 255; brightness >= 0; brightness -= 5) {
//     colors[LED_COUNT - 1] = rgb_color(0, brightness, 0);
//     ledStrip.write(colors, LED_COUNT);
//     delay(30);  // Adjust delay for desired easing effect
//   }

//   // Turn off the last LED
//   colors[LED_COUNT - 1] = rgb_color(0, 0, 0);
//   ledStrip.write(colors, LED_COUNT);
// }

// Easing function to make the animation start fast and slow down
// float easeOut(float t) {
//   return 1 - pow(1 - t, 40);
// }

//////////////////////////////////////////////////////////////////////////////
// MAIN LOOP
//////////////////////////////////////////////////////////////////////////////

void loop() {
 
 Serial.println("going");
 
  randoblink();

  if (ready_to_roll == true) {
    flickerLights_green();
  }

  if (ready_to_roll == false && diResults <= 19) {
    gradLoop();
  }

  if (ready_to_roll == false && diResults >= 20) {
    gradLoopCRIT();
    
  }


  while (0) {
    rollResults();
    delay(250);
  }
  updateButtonStates();
  buttonPress();
  addAccToQueue();
  swing();
}
