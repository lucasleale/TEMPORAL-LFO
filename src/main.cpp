#include <Adafruit_GFX.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <ArduinoTapTempo.h>
#include <Bounce2.h>
#include <Fonts/Font4x5Fixed.h>
#include <Fonts/Font4x7Fixed.h>
#include <Fonts/Font5x5Fixed.h>
#include <Fonts/Font5x7Fixed.h>
#include <Fonts/Font5x7FixedMono.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Lfo.h>
#include <ResponsiveAnalogRead.h>
#include <Wire.h>
// #include <Encoder.h>
#include <elapsedMillis.h>
#include <pio_encoder.h>
elapsedMillis fps;
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
#define SYNC1_OUT_PIN 17
#define SYNC2_OUT_PIN 18
#define SYNC_IN_PIN 26
#define MAX_BPM 340
#define MIN_BPM 40
#define ROW1_WAVE 64  // coordenadas Y para el dibujo de formas de onda
#define ROW2_WAVE 88
#define ROW1_RATIO 68
#define ROW2_RATIO 92
#define TRIGGER_DUTY 200  // 200 * 10khz = 20mS
#define POS 1
#define INV 0
const uint16_t TRIGGER_PERIOD = 200;  // 200 ticks a 10khz * 0.1ms = 20ms

const uint8_t NUM_WAVES = 7;
const uint32_t SAMPLE_RATE = 10000;  // dejar fijo en 10khz o 4khz?
const float SAMPLE_RATE_MS = 1000. / SAMPLE_RATE;
const uint32_t TABLE_SIZE = 4096;                   // largo de tabla de ondas, tal vez lo incremente mas
const uint32_t PWM_RANGE = 4095;                    // (2^n )- 1 //4095 para 12bit, 1023 para 10bit
const uint32_t PWM_FREQ = F_CPU / (PWM_RANGE + 1);  // 61035Hz en 12bit, 244,140Hz en 10bit
const float ALARM_PERIOD = 1000000. / SAMPLE_RATE;  // timer interrupt en microsegundos

Lfo lfo(LFO1_PIN, LFO2_PIN, SAMPLE_RATE, TABLE_SIZE, PWM_RANGE, SYNC1_OUT_PIN, SYNC2_OUT_PIN);
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
ResponsiveAnalogRead potMult1(POT1_PIN, true);
ResponsiveAnalogRead potMult2(POT2_PIN, true);
ArduinoTapTempo tap;
Bounce wave1 = Bounce();
Bounce wave2 = Bounce();
Bounce tapBounce = Bounce();
PioEncoder encoder(2);
const float multipliers[] = {0.25, 0.5, 0.66666, 1., 1.5, 2., 3., 4.};
const uint8_t numMultipliers = (sizeof(multipliers) / sizeof(byte*));
const uint8_t multipliersSync[numMultipliers] = {4, 2, 6, 1, 2, 1, 1, 1};
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

volatile uint32_t counterTicksPulse;
uint32_t counterTicksLoop;

volatile uint32_t counterDivTicksLfo1;
volatile uint32_t pulseTicksLfo1;
volatile float pulsePeriodMsLfo1;
// volatile uint32_t counterTicksLfo2;
volatile uint32_t counterDivTicksLfo2;
volatile uint32_t pulseTicksLfo2;
volatile float pulsePeriodMsLfo2;
volatile uint32_t pulseCounter;
volatile bool gotNewPulse;
bool lastGotNewPulse;
volatile bool pulseConnected = false;
bool lastPulseConnected;
volatile bool newTriggerLfo1 = true;  // para que el reset en el sync externo no corra riesgo de doble trigger.
volatile bool newTriggerLfo2 = true;  // para que el reset en el sync externo no corra riesgo de doble trigger.
volatile uint32_t timeOut = 30000;
volatile uint32_t counterTimeOut;
int multiplier1;
int multiplier2;
volatile float ratioLfo1 = 1.;
volatile float ratioLfo2 = 1.;
volatile uint8_t multiplierSyncLfo1 = 1;
volatile uint8_t multiplierSyncLfo2 = 1;
volatile int encoderValueISR;
bool polarityLfo1 = 0;
bool polarityLfo2 = 0;
float bpm = 120.;
float bpmCore2 = 120.;
bool newBpm = false;
bool newWave = false;
float periodMs = 500;
bool tapState;
byte waveSelector1Core2;
byte waveSelector2Core2;
int multiplier1Core2;
int multiplier2Core2;
void syncPulse() {
  pulseConnected = true;
  counterTimeOut = 0;  // este es el watchdog, resetea a 0 en todos los pulsos
  lfo.enableSync();
  // if(ratioLfo1 == 1){
  //   digitalWrite(SYNC1_OUT_PIN, HIGH); //thru
  //   counterDivTicksLfo1 = 0;

  //}
  if (pulseCounter % 2 == 0) {  // always
    counterTicksPulse = 0;
  } else if (pulseCounter % 2 == 1) {  // always
    // pulsePeriod = counter * 0.5; //creo que lo calculamos fuera del timer, aca y en ratio

    if (counterTicksPulse > 400) {  // que tome como pulso valido si es mayor al umbral de bounce
      pulseTicksLfo1 = counterTicksPulse * ratioLfo1;
      pulsePeriodMsLfo1 = counterTicksPulse * SAMPLE_RATE_MS;
      pulseTicksLfo2 = counterTicksPulse * ratioLfo2;
      // pulsePeriodMsLfo2 = counterTicksPulse * SAMPLE_RATE_MS; //en verdad solo necesito el pulseperiod de uno solo
      lfo.setPeriodMs(0, pulsePeriodMsLfo1);
      lfo.setPeriodMs(1, pulsePeriodMsLfo1);

      gotNewPulse = true;
    }
  }

  if (pulseCounter % int(ceil(ratioLfo1 * multiplierSyncLfo1)) == 0) {  // solo cuando ratio es mayor a 1. o alguna irregular tresillo punti
    // counterTicksPulse = 0;
    //  digitalWrite(17, HIGH);
    counterDivTicksLfo1 = 0;
  }
  if (pulseCounter % int(ceil(ratioLfo2 * multiplierSyncLfo2)) == 0) {  // solo cuando ratio es mayor a 1. o alguna irregular tresillo punti
    // counterTicksPulse = 0;
    //  digitalWrite(17, HIGH);
    counterDivTicksLfo2 = 0;
  }

  pulseCounter++;
}
//// TIMER INTERRUPT PARA LFO Y TICK COUNTER PULSE////

static void alarm_in_us_arm(uint32_t delay_us) {
  uint64_t target = timer_hw->timerawl + delay_us;
  timer_hw->alarm[ALARM_NUM] = (uint32_t)target;
}

static void alarm_irq(void) {
  hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);
  alarm_in_us_arm(ALARM_PERIOD);

  lfo.update();  // codigo lfo

  if (pulseConnected) {
    if (counterDivTicksLfo1 % pulseTicksLfo1 == 0) {  // LFO1
      if (newTriggerLfo1) {
        // digitalWrite(SYNC1_OUT_PIN, HIGH);
        lfo.resetPhase(0);
        // lfo.resetPhase(1);
        newTriggerLfo1 = false;
      }
    } else if (counterDivTicksLfo1 % pulseTicksLfo1 == TRIGGER_DUTY) {
      // digitalWrite(SYNC1_OUT_PIN, LOW);
      newTriggerLfo1 = true;
    }
    if (counterDivTicksLfo2 % pulseTicksLfo2 == 0) {  // LFO1
      if (newTriggerLfo2) {
        // digitalWrite(SYNC1_OUT_PIN, HIGH);
        lfo.resetPhase(1);
        // lfo.resetPhase(1);
        newTriggerLfo2 = false;
      }
    } else if (counterDivTicksLfo2 % pulseTicksLfo2 == TRIGGER_DUTY) {
      // digitalWrite(SYNC1_OUT_PIN, LOW);
      newTriggerLfo2 = true;
    }

    counterTicksPulse++;
    counterDivTicksLfo1++;
    counterDivTicksLfo2++;
    counterTimeOut++;
  }
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

float bpmToMs(float bpm) {
  const float msQuarter = 60000.;
  return (float)msQuarter / (float)bpm;
}
float msToBpm(float period) {
  const float msQuarter = 60000.;
  return (float)msQuarter / (float)period;
}

void updateBpm() {
  if (!pulseConnected) {
    periodMs = bpmToMs(bpm);
    lfo.setPeriodMs(0, periodMs);
    lfo.setPeriodMs(1, periodMs);

    // displayBpm(bpm);
    bpmCore2 = bpm;
    // pushCore2 = true;
    tap.setBPM(bpm);  // para que el tap no salte a cualquier valor, que se vaya ajustando
  }
}
void updateEncoderBpm() {
  static int encoderValue;
  static int lastEncoderValue;
  static int encoderResult;
  encoderValue = encoder.getCount() >> 2;  // dividido 4
  if (encoderValue != lastEncoderValue) {
    if (encoderValue > lastEncoderValue) {
      encoderResult = 1;
      // Serial.println("sube");
    } else {
      // Serial.println("baja");
      encoderResult = -1;
    }
    if (!pulseConnected) {  // que lo refresque si no hay clock externo
      bpm += encoderResult;
    }
    if (bpm >= MAX_BPM) {
      bpm = MAX_BPM;
    } else if (bpm <= MIN_BPM) {
      bpm = MIN_BPM;
    }
    updateBpm();
    lastEncoderValue = encoderValue;
  }
}

void updatePots() {
  potMult1.update();
  potMult2.update();
  static int lastMultiplier1;
  static int lastMultiplier2;
  multiplier1 = map(potMult1.getValue(), 0, 1023, 0, numMultipliers);
  multiplier2 = map(potMult2.getValue(), 0, 1023, 0, numMultipliers);

  // delay(20);
  if (multiplier1 != lastMultiplier1) {
    lastMultiplier1 = multiplier1;
    // oled.setTextSize(1);
    // oled.setCursor(36, 68);
    ratioLfo1 = multipliers[multiplier1];
    // multipliers1Core2 = multipliersOled[multiplier1];
    multiplier1Core2 = multiplier1;
    multiplierSyncLfo1 = multipliersSync[multiplier1];
    // oled.print(multipliersOled[multiplier1]);
    // oled.display();
    lfo.setRatio(0, ratioLfo1);
  }
  if (multiplier2 != lastMultiplier2) {
    lastMultiplier2 = multiplier2;
    // oled.setTextSize(1);
    // oled.setCursor(36, 92);
    // multipliers2Core2 = multipliersOled[multiplier2];
    multiplier2Core2 = multiplier2;
    ratioLfo2 = multipliers[multiplier2];
    multiplierSyncLfo2 = multipliersSync[multiplier2];
    // oled.print(multipliersOled[multiplier2]);
    // oled.display();
    lfo.setRatio(1, ratioLfo2);
  }
}

void updateButtons() {
  wave1.update();
  wave2.update();
  static uint8_t waveSelector1 = 1;
  static uint8_t waveSelector2 = 1;
  if (wave1.rose()) {
    lfo.setWave(0, waveSelector1);
    waveSelector1Core2 = waveSelector1;
    // pushCore2 = true;
    //  displayWave(waveSelector1, ROW1_WAVE);
    waveSelector1++;
    if (waveSelector1 == NUM_WAVES) {
      waveSelector1 = 0;
    }
  }

  if (wave2.rose()) {
    lfo.setWave(1, waveSelector2);
    waveSelector2Core2 = waveSelector2;
    // displayWave(waveSelector2, ROW2_WAVE);
    // pushCore2 = true;
    waveSelector2++;

    if (waveSelector2 == NUM_WAVES) {
      waveSelector2 = 0;
    }
  }
}

void tapTempo() {
  if (!pulseConnected) {
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
      bpmCore2 = bpm;
      // displayBpm(bpm);
    }
  }
}

void setup() {
  // Serial.begin(9600);
  encoder.begin();
  encoder.flip();
  analogWriteFreq(PWM_FREQ);
  analogWriteRange(PWM_RANGE);
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(ENC_PINA, INPUT_PULLUP);
  pinMode(ENC_PINB, INPUT_PULLUP);
  pinMode(LFO1_PIN, OUTPUT);
  pinMode(LFO2_PIN, OUTPUT);
  pinMode(TAP_PIN, INPUT_PULLUP);
  pinMode(SYNC1_OUT_PIN, OUTPUT);
  pinMode(SYNC2_OUT_PIN, OUTPUT);
  pinMode(SYNC_IN_PIN, INPUT_PULLUP);
  wave1.attach(BTN1_PIN, INPUT_PULLUP);
  wave2.attach(BTN2_PIN, INPUT_PULLUP);
  wave1.interval(25);
  wave2.interval(25);
  tapBounce.attach(TAP_PIN, INPUT_PULLUP);
  tapBounce.interval(25);

  lfo.setTriggerPeriod(0, TRIGGER_PERIOD);
  lfo.setTriggerPeriod(1, TRIGGER_PERIOD);
  lfo.setPeriodMs(0, 500);
  lfo.setPeriodMs(1, 500);
  lfo.setRatio(0, 1);
  lfo.setRatio(1, 1);
  lfo.setTriggerPolarity(0, POS);
  lfo.setTriggerPolarity(1, POS);
  alarm_in_us(ALARM_PERIOD);

  attachInterrupt(digitalPinToInterrupt(SYNC_IN_PIN), syncPulse, RISING);
  // oled.setFont(&FreeSans9pt7b);
  // oled.display();
  // oled.clearDisplay();
  // oled.setRotation(1);
  // oled.setTextSize(1);
  // oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

  // displayWave(0, ROW2_WAVE);
  // displayWave(0, ROW1_WAVE);
  // oled.display();
  potMult1.setActivityThreshold(32);
  potMult1.setSnapMultiplier(0.0001);
  potMult2.setActivityThreshold(32);
  potMult2.setSnapMultiplier(0.0001);
  updateButtons();
  updatePots();
  lfo.resetPhase(0);
  lfo.resetPhase(1);
  // displayBpm(120.0);
}

void loop() {
  updatePots();
  updateButtons();
  updateEncoderBpm();
  tapTempo();
  if (gotNewPulse) {
    // Serial.print(ratioLfo1);
    // Serial.print("  ");
    // Serial.println(60000. /(counterTicksPulse * SAMPLE_RATE_MS));
    // Serial.print(counterTicksPulse);
    // Serial.print("  ");

    // Serial.println(60000. / (counterTicksPulse * SAMPLE_RATE_MS));
    // noInterrupts();
    counterTicksLoop = counterTicksPulse;
    // rp2040.fifo.push(counterTicksLoop);

    // interrupts();
    Serial.println(counterTicksLoop);
    if (counterTicksPulse < 400) {  //
      // Serial.println("disconnected");
      //  pulseConnected = false;
      //  lfo.disableSync();
      //  updateBpm();
    }
    // displayBpm(msToBpm(pulsePeriodMsLfo1));

    gotNewPulse = false;
  }
  if (lastGotNewPulse != gotNewPulse) {
    updateBpm();

    lastGotNewPulse = gotNewPulse;
  }
  if (pulseConnected && counterTimeOut > timeOut) {
    pulseConnected = false;
    lfo.disableSync();
    updateBpm();
  }
  if (lastPulseConnected != pulseConnected) {
    if (pulseConnected) {
      // displayBpm(0);  // 0 == ext clock
      bpmCore2 = 0;
      // newBpm = true;
    }
    lastPulseConnected = pulseConnected;
  }

  // Serial.println(counterTimeOut);
  // delay(100);
  // Serial.println(counterTimeOut);
  // encoder.reset();
  // Serial.println(encoder.getCount() / 4);
  // delay(10);
  // rp2040.fifo.push(500);
  // delay(30);

  // delay(33);
}

//////CORE 2 PARA OLED Y NEOPIXEL
void displayBpm(float bpm) {
  oled.setCursor(0, 8);
  oled.fillRect(0, 0, 63, 22, BLACK);
  oled.setTextSize(2);
  if (bpm != 0) {  // si no es bpm 0 es bpm, si no es EXT.
    oled.print(bpm, 1);
  } else {
    oled.print("EXT");
  }
  oled.display();
}

void displayRatio(int multiplier, byte row) {
  oled.setTextSize(1);
  oled.setCursor(36, row);
  oled.print(multipliersOled[multiplier]);
  oled.display();
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
void setup1() {
  Serial.begin(9600);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    // Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  oled.display();
  oled.clearDisplay();
  oled.setRotation(1);
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  displayWave(0, ROW1_WAVE);
  displayWave(0, ROW2_WAVE);
  displayBpm(120.0);
}

float lastBpmCore2;
byte lastWaveSelector1Core2;
byte lastWaveSelector2Core2;
int lastMultiplier1Core2;
int lastMultiplier2Core2;
void loop1() {
  // uint32_t counterTicksPulseCore = rp2040.fifo.pop();
  // Serial.println(counterTicksPulseCore);
  // delay(20);
  if (lastBpmCore2 != bpmCore2) {
    displayBpm(bpmCore2);
    lastBpmCore2 = bpmCore2;
  }

  if (lastWaveSelector1Core2 != waveSelector1Core2) {
    displayWave(waveSelector1Core2, ROW1_WAVE);
    lastWaveSelector1Core2 = waveSelector1Core2;
  }

  if (lastWaveSelector2Core2 != waveSelector2Core2) {
    displayWave(waveSelector2Core2, ROW2_WAVE);
    lastWaveSelector2Core2 = waveSelector2Core2;
  }
  if (lastMultiplier1Core2 != multiplier1Core2) {
    displayRatio(multiplier1Core2, ROW1_RATIO);
    lastMultiplier1Core2 = multiplier1Core2;
  }
  if (lastMultiplier2Core2 != multiplier2Core2) {
    displayRatio(multiplier2Core2, ROW2_RATIO);
    lastMultiplier2Core2 = multiplier2Core2;
  }
}