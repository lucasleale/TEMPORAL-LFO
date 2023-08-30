#include <Adafruit_GFX.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <ArduinoTapTempo.h>
#include <Bounce2.h>
#include <Lfo.h>
#include <ResponsiveAnalogRead.h>
#include <Wire.h>

#include "pico/stdlib.h"
#include "settings.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

#define SCREEN_ADDRESS 0x3C  // CHEQUEAR ADDRESS, PUEDE SER 0x3C o 0x3D

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
#define ROW1_WAVE 64  // coordenadas Y para el dibujo de formas de onda
#define ROW2_WAVE 88

const uint8_t NUM_WAVES = 7;
const uint32_t SAMPLE_RATE = 4000;                  // dejar fijo en 4khz
const uint32_t TABLE_SIZE = 4096;                   // largo de tabla de ondas, tal vez lo incremente mas
const uint32_t PWM_RANGE = 4095;                    // (2^n )- 1 //4095 para 12bit, 1023 para 10bit
const uint32_t PWM_FREQ = F_CPU / (PWM_RANGE + 1);  // 61035Hz en 12bit, 244,140Hz en 10bit
const float ALARM_PERIOD = 1000000. / SAMPLE_RATE;  // timer interrupt en microsegundos

Lfo lfo(LFO1_PIN, LFO2_PIN, SAMPLE_RATE, TABLE_SIZE, PWM_RANGE);
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
ResponsiveAnalogRead potMult1(POT1_PIN, true);
ResponsiveAnalogRead potMult2(POT2_PIN, true);
ArduinoTapTempo tap;
Bounce wave1 = Bounce();
Bounce wave2 = Bounce();
Bounce tapBounce = Bounce();

const float multipliers[] = {0.25, 0.5, 0.66666, 1., 1.5, 2., 3., 4.};
const uint8_t numMultipliers = (sizeof(multipliers) / sizeof(byte*));
const char* multipliersOled[numMultipliers] = {"1/16", "1/8 ", "1/4t", "1/4 ", "1/4*", "1/2 ", "1/2*", "1   "};

const uint8_t oledX = 16;
const float oledCycles = 2;
const uint8_t oledWidth = oledX * oledCycles;
const uint8_t sineOled[oledX] = {8, 5, 2, 1, 0, 1, 2, 5, 8, 11, 14, 15, 16, 15, 14, 11};
const uint8_t sawUpOled[oledX] = {7, 6, 5, 4, 3, 2, 1, 0, 15, 14, 13, 12, 11, 10, 9, 8};
const uint8_t sawDownOled[oledX] = {8, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3, 4, 5, 6, 7};
const uint8_t triOled[oledX] = {8, 6, 4, 2, 0, 2, 4, 6, 8, 10, 12, 14, 16, 14, 12, 10};
const uint8_t sqrOled[oledX] = {15, 0, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 15, 15, 15, 15};
const uint8_t randomOled[] = {8, 8, 8, 8, 10, 10, 10, 10,
                              4, 4, 4, 4, 12, 12, 12, 12,
                              9, 9, 9, 9, 15, 15, 15, 15,
                              2, 2, 2, 2, 7, 7, 7, 7};

const uint8_t* wavetablesOled[] = {sineOled,
                                   sawUpOled,
                                   sawDownOled,
                                   triOled,
                                   sqrOled,
                                   randomOled};

int multiplier1;
int multiplier2;
volatile int encoderValueISR;

float bpm = 120.;
float periodMs = 500;
bool tapState;

//// TIMER INTERRUPT PARA LFO ////
static void alarm_in_us_arm(uint32_t delay_us) {
  uint64_t target = timer_hw->timerawl + delay_us;
  timer_hw->alarm[ALARM_NUM] = (uint32_t)target;
}

static void alarm_irq(void) {
  hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);
  alarm_in_us_arm(ALARM_PERIOD);

  lfo.update();  // codigo lfo
}

static void alarm_in_us(uint32_t delay_us) {
  hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
  irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
  irq_set_enabled(ALARM_IRQ, true);
  alarm_in_us_arm(delay_us);
}
//// FIN TIMER INTERRUPT PARA LFO ////

void encoderInterrupt() {

  static uint8_t seqStore;
  static uint8_t pinPair;
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
    tap.setBPM(bpm);  // para que el tap no salte a cualquier valor, que se vaya ajustando
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
    oled.setTextSize(1);
    oled.setCursor(36, 68);
    oled.print(multipliersOled[multiplier1]);
    oled.display();
    lfo.setRatio(0, multipliers[multiplier1]);
  }
  if (multiplier2 != lastMultiplier2) {
    lastMultiplier2 = multiplier2;
    oled.setTextSize(1);
    oled.setCursor(36, 92);
    oled.print(multipliersOled[multiplier2]);
    oled.display();
    lfo.setRatio(1, multipliers[multiplier2]);
  }
}
void displayWave(byte wave, byte row) {
  oled.fillRect(0, row - 1, oledWidth, 18, BLACK);  // row-1 por el borde que le falta a fillrect
  uint8_t* wavetables;                              // pointer al array de wavetablesoleds
  if (wave != 6) {                                  // si no es hold, cambiar el 6 por un macro para editar mas facil adelante
    wavetables = (uint8_t*)wavetablesOled[wave];    // scanea la memoria segun indice wave
    for (int i = 0; i < oledWidth; i++) {
      if (wave != 5) {  // si no es random que haga wrap en cada ciclo
        oled.drawLine(i, wavetables[i % oledX] + row, i, wavetables[(i + 1) % oledX] + row, WHITE);
      } else {
        oled.drawLine(i, wavetables[i % oledWidth] + row, i, wavetables[(i + 1) % oledWidth] + row, WHITE);
      }
    }
  } else {
    oled.drawLine(0, row, oledWidth - 1, row, WHITE);
  }
  oled.display();
}

void updateButtons() {
  wave1.update();
  wave2.update();
  static uint8_t waveSelector1 = 1;
  static uint8_t waveSelector2 = 1;
  if (wave1.rose()) {
    lfo.setWave(0, waveSelector1);
    displayWave(waveSelector1, ROW1_WAVE);
    waveSelector1++;
    if (waveSelector1 == NUM_WAVES) {
      waveSelector1 = 0;
    }
  }

  if (wave2.rose()) {
    lfo.setWave(1, waveSelector2);
    displayWave(waveSelector2, ROW2_WAVE);
    waveSelector2++;

    if (waveSelector2 == NUM_WAVES) {
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
  analogWriteFreq(PWM_FREQ);
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
  lfo.setPeriodMs(0, 500);
  lfo.setPeriodMs(1, 500);
  lfo.setRatio(0, 1);
  lfo.setRatio(1, 1);
  alarm_in_us(ALARM_PERIOD);
  attachInterrupt(digitalPinToInterrupt(ENC_PINA), encoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PINB), encoderInterrupt, CHANGE);
  oled.display();
  oled.clearDisplay();
  oled.setRotation(1);
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

  displayWave(0, ROW2_WAVE);
  displayWave(0, ROW1_WAVE);
  oled.display();
  potMult1.setActivityThreshold(32);
  potMult1.setSnapMultiplier(0.0001);
  potMult2.setActivityThreshold(32);
  potMult2.setSnapMultiplier(0.0001);
  updateButtons();
  updatePots();
  lfo.resetPhase(0);
  lfo.resetPhase(1);
}

void loop() {
  updatePots();
  updateButtons();
  updateEncoderBpm();
  tapTempo();
}
