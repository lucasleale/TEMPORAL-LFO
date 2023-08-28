
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <ArduinoTapTempo.h>
#include <Bounce2.h>
#include <Lfo.h>
#include <ResponsiveAnalogRead.h>
#include <Wire.h>

#include "pico/stdlib.h"
#include "settings.h"
// #include "wavetables.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

#define BUILTIN_LED 25
#define ALARM_NUM 1
#define ALARM_IRQ TIMER_IRQ_1
#define POT1_PIN 27
#define POT2_PIN 28
#define BTN1_PIN 14
#define BTN2_PIN 15
#define TAP_PIN 12
#define ENC_PINA 3
#define ENC_PINB 2
#define LFO1_PIN 20
#define LFO2_PIN 21
#define MAX_BPM 340
#define MIN_BPM 40
#define LFO1 0
#define LFO2 1

const uint32_t SAMPLE_RATE = 4000;
const uint32_t TABLE_SIZE = 4096;
const uint32_t PWM_RANGE = 4095;
const float ALARM_PERIOD = 1000000. / SAMPLE_RATE;  // timer interrupt en microsegundos

Lfo lfo(LFO1_PIN, LFO2_PIN, SAMPLE_RATE, TABLE_SIZE);
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
ResponsiveAnalogRead potMult1(POT1_PIN, true);
ResponsiveAnalogRead potMult2(POT2_PIN, true);
ArduinoTapTempo tap;
Bounce wave1 = Bounce();
Bounce wave2 = Bounce();
Bounce tapBounce = Bounce();
const float multipliers[] = {0.25, 0.5, 1., 1.5, 2., 3., 4.};
const uint8_t numMultipliers = (sizeof(multipliers) / sizeof(byte*));
const char* multipliersOled[numMultipliers] = {"1/16", "1,8", "1/4", "1/2", "1"};
const uint8_t numWaves = 7;

int multiplier1;
int multiplier2;
volatile int encoderValueISR;
volatile byte pinPair;
volatile byte seqStore;
float bpm = 120.;
float periodMs = 500;
volatile float bpmTest = 500;
bool tapState;
//// TIMER INTERRUPT PARA LFO ////

static void alarm_in_us_arm(uint32_t delay_us) {
  uint64_t target = timer_hw->timerawl + delay_us;
  timer_hw->alarm[ALARM_NUM] = (uint32_t)target;
}

static void alarm_irq(void) {
  hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);
  alarm_in_us_arm(ALARM_PERIOD);

  lfo.update();
}

static void alarm_in_us(uint32_t delay_us) {
  hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
  irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
  irq_set_enabled(ALARM_IRQ, true);
  alarm_in_us_arm(delay_us);
}
//// FIN TIMER INTERRUPT PARA LFO ////

void encoderInterrupt() {
  // static byte encoderPinA = digitalRead(ENC_PINA);
  // static byte encoderPinB = digitalRead(ENC_PINB);
  pinPair = (digitalRead(ENC_PINB) << 1) | digitalRead(ENC_PINA);
  encoderValueISR = 0;
  if ((seqStore & 0x3) != pinPair) {
    seqStore = seqStore << 2;
    seqStore |= pinPair;
    if (seqStore == 0b10000111) {
      encoderValueISR = -1;
    }
    if (seqStore == 0b01001011) {
      encoderValueISR = 1;
    }
  }
}

void displayBpm(float bpm) {
  oled.setCursor(0, 8);
  oled.setTextSize(2);
  oled.print(bpm, 1);
  oled.display();
}
float bpmToMs(float bpm) {
  const float msQuarter = 60000.;
  return (float)msQuarter / (float)bpm;
}
float msToBpm(float period) {
  const float msQuarter = 60000.;
  return (float)msQuarter / (float)period;
}
void updateEncoderBpm() {
  static int encoderValue = 1;
  static int lastEncoderValue = 1;

  encoderValue = encoderValueISR;
  if (encoderValue != lastEncoderValue) {
    lastEncoderValue = encoderValue;
    bpm += encoderValue;
    if (bpm >= MAX_BPM) {
      bpm = MAX_BPM;
    } else if (bpm <= MIN_BPM) {
      bpm = MIN_BPM;
    }

    periodMs = bpmToMs(bpm);

    lfo.setPeriodMs(0, periodMs);
    lfo.setPeriodMs(1, periodMs);

    displayBpm(bpm);
  }
}
void updatePots() {
  potMult1.update();
  potMult2.update();
  static int lastMultiplier1;
  static int lastMultiplier2;
  multiplier1 = map(potMult1.getValue(), 0, 1023, 0, numMultipliers);
  multiplier2 = map(potMult2.getValue(), 0, 1023, 0, numMultipliers);

  if (multiplier1 != lastMultiplier1) {
    lastMultiplier1 = multiplier1;
    // Serial.println(multiplier1);
    // lfo1.setPeriodMs(bpmToMs(bpm*multipliers[multiplier1])); //cambiar
    Serial.println(multipliers[multiplier1]);
    lfo.setRatio(0, multipliers[multiplier1]);
    // esta garcha
  }
  if (multiplier2 != lastMultiplier2) {
    lastMultiplier2 = multiplier2;
    Serial.println(multipliers[multiplier2]);
    // lfo2.setPeriodMs(bpmToMs(bpm*multipliers[multiplier2]));
    lfo.setRatio(1, multipliers[multiplier2]);
  }
}
void updateButtons() {
  wave1.update();
  wave2.update();
  static uint8_t waveSelector1;
  static uint8_t waveSelector2;
  if (wave1.rose()) {
    lfo.setWave(0, waveSelector1);
    waveSelector1++;
    if (waveSelector1 == numWaves) {
      waveSelector1 = 0;
    }
  }

  if (wave2.rose()) {
    lfo.setWave(1, waveSelector2);
    waveSelector2++;

    if (waveSelector2 == numWaves) {
      waveSelector2 = 0;
    }
  }
}

void tapTempo() {
  tapBounce.update();
  tapState = tapBounce.read();
  tap.update(tapState);

  if (tapBounce.fell()) {
    periodMs = tap.getBeatLength();
    Serial.println(tap.getBeatLength());
    lfo.resetPhase(0);
    lfo.resetPhase(1);
    lfo.setPeriodMs(0, periodMs);
    lfo.setPeriodMs(1, periodMs);
    bpm = msToBpm(periodMs);
    displayBpm(bpm);
  }
}
void setup() {
  Serial.begin(9600);
  analogWriteFreq(60000);
  analogWriteRange(PWM_RANGE);
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(ENC_PINA, INPUT_PULLUP);
  pinMode(ENC_PINB, INPUT_PULLUP);
  pinMode(LFO1_PIN, OUTPUT);
  pinMode(LFO2_PIN, OUTPUT);
  pinMode(TAP_PIN, INPUT_PULLUP);
  wave1.attach(BTN1_PIN, INPUT_PULLUP);
  wave2.attach(BTN2_PIN, INPUT_PULLUP);
  wave1.interval(25);
  wave2.interval(25);
  tapBounce.attach(TAP_PIN, INPUT_PULLUP);
  tapBounce.interval(25);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  lfo.setPeriodMs(0, 1000);
  lfo.setPeriodMs(1, 1000);

  lfo.setWave(0, 3);
  lfo.setWave(1, 3);
  lfo.setRatio(0, 4);
  lfo.setRatio(1, 1);
  lfo.resetPhase(0);
  lfo.resetPhase(1);
  delay(200);
  alarm_in_us(ALARM_PERIOD);

  attachInterrupt(digitalPinToInterrupt(ENC_PINA), encoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PINB), encoderInterrupt, CHANGE);

  oled.display();
  oled.clearDisplay();
  oled.setRotation(1);
  oled.setTextSize(2);
  oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

  oled.setCursor(40, 64);
  oled.setTextSize(1);
  oled.print("1/8");
  oled.setCursor(40, 88);
  oled.print("1/16");
  // oled.println("120.0");
  oled.display();
  potMult1.setActivityThreshold(32);
  potMult1.setSnapMultiplier(0.0001);
  potMult2.setActivityThreshold(32);
  potMult2.setSnapMultiplier(0.0001);
}

void loop() {
  if (Serial.available()) {
    char serialInput = Serial.read();
    if (serialInput == 'r') {
      // lfo1.resetPhase();
      // lfo2.resetPhase();
    }
    if (serialInput == 'w') {
      // lfo1.setPeriodMs(bpmToMs(120.) * 2.);
    }
    if (serialInput == 's') {
      // lfo1.setPeriodMs(bpmToMs(120.) * 1.);
    }
  }

  updatePots();
  updateButtons();
  updateEncoderBpm();
  tapTempo();
}
