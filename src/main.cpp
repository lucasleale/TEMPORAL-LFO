// branch testdisplay

#include <Adafruit_GFX.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_NeoPixel.h>
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

#include "fscale.h"
#include "pico/stdlib.h"
#include "settings.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C  // CHEQUEAR ADDRESS, PUEDE SER 0x3C o 0x3D

#define LED_REFRESH 33  // 1000/33 30fps
#define LED_LFO1 1
#define LED_LFO2 2
#define LED_PIN 16
#define NUM_LEDS 6
#define BUILTIN_LED 25
#define ALARM_NUM 1
#define ALARM_IRQ TIMER_IRQ_1
#define POT1_PIN 27
#define POT2_PIN 28
#define BTN1_PIN 14
#define BTN2_PIN 15
#define TAP_PIN 12
#define TAP_JACK 22
#define ENC_PINA 3
#define ENC_PINB 2
#define LFO1_PIN 20
#define LFO2_PIN 21
#define SYNC1_OUT_PIN 17
#define SYNC2_OUT_PIN 18
#define CLOCK_OUT_PIN 19
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
const uint32_t SAMPLE_RATE = 4000;  // dejar fijo en 10khz o 4khz?
const float SAMPLE_RATE_MS = 1000. / SAMPLE_RATE;
const uint32_t TABLE_SIZE = 4096;                   // largo de tabla de ondas, tal vez lo incremente mas
const uint32_t PWM_RANGE = 1023;                    // (2^n )- 1 //4095 para 12bit, 1023 para 10bit
const uint32_t PWM_FREQ = F_CPU / (PWM_RANGE + 1);  // 61035Hz en 12bit, 244,140Hz en 10bit
const float ALARM_PERIOD = 1000000. / SAMPLE_RATE;  // timer interrupt en microsegundos

Lfo lfo(LFO1_PIN, LFO2_PIN, SAMPLE_RATE, TABLE_SIZE, PWM_RANGE, SYNC1_OUT_PIN, SYNC2_OUT_PIN);
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
ResponsiveAnalogRead potMult1(POT1_PIN, true);
ResponsiveAnalogRead potMult2(POT2_PIN, true);
Adafruit_NeoPixel leds(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
ArduinoTapTempo tap;
Bounce wave1 = Bounce();
Bounce wave2 = Bounce();
Bounce tapBtn = Bounce();
Bounce tapExt = Bounce();
PioEncoder encoder(2);
elapsedMillis btnWaveTime1;
elapsedMillis btnWaveTime2;
elapsedMillis ledsFps;
// prototypes
void updateButtons();
void updatePots();
void updateEncoderBpm();
void tapTempo();
void updateBpm();
void updateEncoderBpm();
void updateLfoLeds();
void updateTempoLed();
float msToBpm(float period);
float bpmToMs(float bpm);
void syncPulse();
static void alarm_in_us_arm(uint32_t delay_us);
static void alarm_irq(void);
static void alarm_in_us(uint32_t delay_us);

const float multipliers[] = {4., 3., 2., 1.5, 1., 0.6666, 0.5, 0.25};  // era 0.25, 0.5, 0.66666, 1., 1.5, 2, 3., 4.
const uint8_t numMultipliers = (sizeof(multipliers) / sizeof(byte*));
const uint8_t multipliersSync[numMultipliers] = {1, 1, 1, 2, 1, 6, 2, 4};  // era 4,2,6,1,2,1,1,1
const char* multipliersOled[numMultipliers] = {"1   ", "1/2*", "1/2 ", "1/4*", "1/4 ", "1/4t", "1/8 ", "1/16"};
// era "1/16", "1/8 ", "1/4t", "1/4 ", "1/4*", "1/2 ", "1/2*", "1   "
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
int periodFreeRunning1Core2;
int periodFreeRunning2Core2;
bool btnWaveHold1;
bool btnWaveHold2;
bool isFreeRunning1 = 0;
bool isFreeRunning2 = 0;
bool isFreeRunning1Core2 = 0;
bool isFreeRunning2Core2 = 0;
int periodFreeRunning1;
int periodFreeRunning2;

/// para display
bool flagBpm;
bool flagRatio;
bool flagRatio2;
bool flagWave;
bool flagWave2;
bool flagFreq;
bool flagFreq2;

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
  pinMode(TAP_JACK, INPUT_PULLUP);
  pinMode(SYNC1_OUT_PIN, OUTPUT);
  pinMode(SYNC2_OUT_PIN, OUTPUT);
  pinMode(SYNC_IN_PIN, INPUT_PULLUP);
  wave1.attach(BTN1_PIN, INPUT_PULLUP);
  wave2.attach(BTN2_PIN, INPUT_PULLUP);
  wave1.interval(25);
  wave2.interval(25);
  tapBtn.attach(TAP_PIN, INPUT_PULLUP);
  tapBtn.interval(25);
  tapExt.attach(TAP_JACK, INPUT_PULLUP);
  tapExt.interval(25);
  lfo.setTriggerPeriod(0, TRIGGER_PERIOD);
  lfo.setTriggerPeriod(1, TRIGGER_PERIOD);
  lfo.setPeriodMs(0, 500);
  lfo.setPeriodMs(1, 500);
  lfo.setPeriodMsClock(500);
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
  leds.begin();
  leds.setPixelColor(0, leds.Color(100, 30, 100));
  leds.setPixelColor(1, leds.Color(30, 100, 100));
  leds.setPixelColor(2, leds.Color(100, 100, 30));
  leds.setPixelColor(3, leds.Color(100, 30, 100));
  leds.setPixelColor(4, leds.Color(30, 100, 100));
  leds.setPixelColor(5, leds.Color(100, 100, 30));
  leds.show();
  updateButtons();
  updatePots();
  lfo.resetPhase(0);
  lfo.resetPhase(1);
  lfo.resetPhaseMaster();
  // displayBpm(120.0);
}
void syncPulse() {
  pulseConnected = true;
  lfo.resetPhaseMaster();  // reseteamos el master clock interno solo con los pulsos
                           // antes el reset era igual que los lfos, entonces se triggeaba el clock out con cada syncout
  counterTimeOut = 0;      // este es el watchdog, resetea a 0 en todos los pulsos
  if (!isFreeRunning1) {   // el sync lo activa solo si no esta en free
    lfo.enableSync(0);
  }  // mover esto a otro lado...
  if (!isFreeRunning2) {
    lfo.enableSync(1);
  }
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
      if (!isFreeRunning1) {  // lo mismo aca con el free running
        lfo.setPeriodMs(0, pulsePeriodMsLfo1);
      }
      if (!isFreeRunning2) {
        lfo.setPeriodMs(1, pulsePeriodMsLfo1);
      }
      lfo.setPeriodMsClock(periodMs);

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

  lfo.update();

  if (pulseConnected) {
    if (counterDivTicksLfo1 % pulseTicksLfo1 == 0) {  // LFO1
      if (newTriggerLfo1) {
        // digitalWrite(SYNC1_OUT_PIN, HIGH);
        if (!isFreeRunning1) {
          lfo.resetPhase(0);
        }
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
        if (!isFreeRunning2) {
          lfo.resetPhase(1);
        }
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
void loop() {
  updatePots();
  updateButtons();
  updateEncoderBpm();
  tapTempo();
  updateLfoLeds();
  updateTempoLed();
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
    // Serial.println(counterTicksLoop);
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
    lfo.disableSync(0);
    lfo.disableSync(1);
    lfo.resetPhase(0);
    lfo.resetPhase(1);
    lfo.resetPhaseMaster();
    updateBpm();
  }
  if (lastPulseConnected != pulseConnected) {
    if (pulseConnected) {
      // displayBpm(0);  // 0 == ext clock
      bpmCore2 = 0;
      flagBpm = true;
      // newBpm = true;
      Serial.println("pulseConnected");
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
void updatePots() {
  potMult1.update();
  potMult2.update();
  static int lastMultiplier1 = -1;
  static int lastMultiplier2 = -1;

  multiplier1 = map(potMult1.getValue(), 0, 1023, 0, numMultipliers);
  multiplier2 = map(potMult2.getValue(), 0, 1023, 0, numMultipliers);

  // delay(20);

  if (multiplier1 != lastMultiplier1) {
    lastMultiplier1 = multiplier1;
    // oled.setTextSize(1);
    // oled.setCursor(36, 68);
    ratioLfo1 = multipliers[multiplier1];
    // multipliers1Core2 = multipliersOled[multiplier1];

    multiplierSyncLfo1 = multipliersSync[multiplier1];
    // oled.print(multipliersOled[multiplier1]);
    // oled.display();
    if (!isFreeRunning1) {  // si no esta en freerunning es ratio
      multiplier1Core2 = multiplier1;
      flagRatio = true;
      lfo.setRatio(0, ratioLfo1);
    }
  }
  periodFreeRunning1 = fscale(potMult1.getValue(), 3, 1023, 1000, 100, 7);

  if (isFreeRunning1) {
    if (potMult1.hasChanged()) {
      lfo.setPeriodMs(0, periodFreeRunning1);
      periodFreeRunning1Core2 = periodFreeRunning1;
      flagFreq = true;
      Serial.print(potMult1.getValue());
      Serial.print("   ");
      Serial.println(periodFreeRunning1);
    }
  }

  if (multiplier2 != lastMultiplier2) {
    lastMultiplier2 = multiplier2;
    // oled.setTextSize(1);
    // oled.setCursor(36, 92);
    // multipliers2Core2 = multipliersOled[multiplier2];

    ratioLfo2 = multipliers[multiplier2];

    multiplierSyncLfo2 = multipliersSync[multiplier2];
    // oled.print(multipliersOled[multiplier2]);
    // oled.display();
    if (!isFreeRunning2) {  // si no esta en freerunning es ratio
      multiplier2Core2 = multiplier2;
      flagRatio2 = true;
      lfo.setRatio(1, ratioLfo2);
    }
  }
  periodFreeRunning2 = fscale(potMult2.getValue(), 3, 1023, 1000, 100, 7);

  if (isFreeRunning2) {
    if (potMult2.hasChanged()) {
      // periodFreeRunning2Core2 = periodFreeRunning2;
      periodFreeRunning2Core2 = periodFreeRunning2;
      flagFreq2 = true;
      Serial.println(periodFreeRunning2);
      lfo.setPeriodMs(1, periodFreeRunning2);
    }
  }
}

void updateButtons() {
  wave1.update();
  wave2.update();
  static uint8_t waveSelector1 = 1;
  static uint8_t waveSelector2 = 1;

  if (wave1.rose() && btnWaveTime1 < 1000) {
    Serial.println("wave");
    lfo.setWave(0, waveSelector1);
    waveSelector1Core2 = waveSelector1;
    flagWave = true;
    // pushCore2 = true;
    //  displayWave(waveSelector1, ROW1_WAVE);
    waveSelector1++;
    if (waveSelector1 == NUM_WAVES) {
      waveSelector1 = 0;
    }
  }
  if (wave1.fell()) {
    btnWaveTime1 = 0;
    btnWaveHold1 = true;
  }
  if (wave1.read() == LOW && btnWaveTime1 >= 1000 && btnWaveHold1) {
    isFreeRunning1 = !isFreeRunning1;
    isFreeRunning1Core2 = isFreeRunning1;
    if (isFreeRunning1) {
      lfo.disableSync(0);
      // updatePots();
      Serial.println("free running1");
      flagFreq = true;
      lfo.turnFreeRunning(0, isFreeRunning1);
      lfo.setPeriodMs(0, periodFreeRunning1);

    } else {
      multiplier1 = map(potMult1.getValue(), 0, 1023, 0, numMultipliers);
      ratioLfo1 = multipliers[multiplier1];
      multiplier1Core2 = multiplier1;
      multiplierSyncLfo1 = multipliersSync[multiplier1];
      Serial.println("wave1");
      flagRatio = true;
      lfo.setPeriodMs(0, periodMs);
      lfo.setRatio(0, ratioLfo1);
      lfo.turnFreeRunning(0, isFreeRunning1);
    }
    btnWaveHold1 = false;
  }

  if (wave2.rose() && btnWaveTime2 < 1000) {
    lfo.setWave(1, waveSelector2);
    waveSelector2Core2 = waveSelector2;
    flagWave2 = true;
    // displayWave(waveSelector2, ROW2_WAVE);
    // pushCore2 = true;
    waveSelector2++;

    if (waveSelector2 == NUM_WAVES) {
      waveSelector2 = 0;
    }
  }
  if (wave2.fell()) {
    btnWaveTime2 = 0;
    btnWaveHold2 = true;
  }
  if (wave2.read() == LOW && btnWaveTime2 >= 1000 && btnWaveHold2) {
    isFreeRunning2 = !isFreeRunning2;
    isFreeRunning2Core2 = isFreeRunning2;
    lfo.disableSync(1);
    if (isFreeRunning2) {
      flagFreq2 = true;
      Serial.println("free running2");
      lfo.turnFreeRunning(1, isFreeRunning2);
      lfo.setPeriodMs(1, periodFreeRunning2);

    } else {
      multiplier2 = map(potMult2.getValue(), 0, 1023, 0, numMultipliers);
      ratioLfo2 = multipliers[multiplier2];
      multiplierSyncLfo2 = multipliersSync[multiplier2];
      multiplier2Core2 = multiplier2;
      flagRatio2 = true;
      Serial.println("wave2");
      lfo.setPeriodMs(1, periodMs);
      lfo.setRatio(1, ratioLfo2);
      lfo.turnFreeRunning(1, isFreeRunning2);
    }
    btnWaveHold2 = false;
  }
}

void tapTempo() {
  if (!pulseConnected) {
    tapBtn.update();
    tapState = tapBtn.read() ^ tapExt.read();  // deberia ser OR pero como es invertido usamos XOR.
    tap.update(tapState);

    if (tapBtn.fell() || tapExt.fell()) {
      periodMs = tap.getBeatLength();

      lfo.resetPhase(0);
      lfo.resetPhase(1);
      lfo.resetPhaseMaster();
      if (!isFreeRunning1) {
        lfo.setPeriodMs(0, periodMs);
      }
      if (!isFreeRunning2) {
        lfo.setPeriodMs(1, periodMs);
      }
      lfo.setPeriodMsClock(periodMs);
      bpm = msToBpm(periodMs);
      bpmCore2 = bpm;
      flagBpm = true;
      // displayBpm(bpm);
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
    lfo.setPeriodMsClock(periodMs);
    // displayBpm(bpm);
    bpmCore2 = bpm;
    flagBpm = true;
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
    bpm = trunc(bpm);  // ignoremos decimales
    updateBpm();
    lastEncoderValue = encoderValue;
  }
}

void updateLfoLeds() {
  uint8_t ledLfo1 = lfo.getLfoValues(0);
  uint8_t ledLfo2 = lfo.getLfoValues(1);
  if (ledsFps > LED_REFRESH) {
    ledsFps = 0;
    // leds.setPixelColor(5, leds.Color(random(256), 100, 30));

    leds.setPixelColor(LED_LFO1, leds.Color(ledLfo1, 0, 0));
    leds.setPixelColor(LED_LFO2, leds.Color(ledLfo2, 0, 0));
    leds.show();
  }
}

void updateTempoLed() {
  bool ledTempo = lfo.getClockOut();
  static bool ledTempoLast;
  if (ledTempo != ledTempoLast) {
    if (pulseConnected) {
      leds.setPixelColor(0, leds.Color(0, 255 * ledTempo, 0));
    } else {
      leds.setPixelColor(0, leds.Color(0, 0, 255 * ledTempo));
    }
    leds.show();
    ledTempoLast = ledTempo;
    // Serial.println(ledTempo);
  }
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
  // flagBpm = false;
}

void displayRatio(int multiplier, byte row) {
  oled.setTextSize(1);
  oled.setCursor(36, row);
  oled.print(multipliersOled[multiplier]);
  oled.display();
  // flagRatio = false;
}

void displayFreq(int period, byte row) {
  oled.setTextSize(1);
  oled.setCursor(36, row);
  oled.fillRect(35, row - 1, 35, 14, BLACK);
  float freq = 1000. / period;
  if (period < 1000) {
    oled.print(freq, 1);
  } else {
    oled.print(freq, 2);
  }
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
  // flagWave = false;
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
int lastPeriodFreeRunning1Core2;
int lastPeriodFreeRunning2Core2;
void loop1() {
  // uint32_t counterTicksPulseCore = rp2040.fifo.pop();
  // Serial.println(counterTicksPulseCore);
  // delay(20);
  // if (lastBpmCore2 != bpmCore2) {
  if (flagBpm) {
    displayBpm(bpmCore2);
    // lastBpmCore2 = bpmCore2;
    flagBpm = false;
  }
  //}

  // if (lastWaveSelector1Core2 != waveSelector1Core2) {
  if (flagWave) {
    displayWave(waveSelector1Core2, ROW1_WAVE);
    lastWaveSelector1Core2 = waveSelector1Core2;
    //}

    // if (lastWaveSelector2Core2 != waveSelector2Core2) {

    lastWaveSelector2Core2 = waveSelector2Core2;
    flagWave = false;
  }
  if (flagWave2) {
    displayWave(waveSelector2Core2, ROW2_WAVE);
    flagWave2 = false;
  }
  //}

  // if (isFreeRunning1Core2) {
  if (flagFreq) {
    // if (lastPeriodFreeRunning1Core2 != periodFreeRunning1Core2) {
    displayFreq(periodFreeRunning1Core2, ROW1_RATIO);
    lastPeriodFreeRunning1Core2 = periodFreeRunning1Core2;

    lastPeriodFreeRunning2Core2 = periodFreeRunning2Core2;
    flagFreq = false;
  }
  if (flagFreq2) {
    displayFreq(periodFreeRunning2Core2, ROW2_RATIO);
    flagFreq2 = false;
  }
  //}
  //} else {
  // if (lastMultiplier1Core2 != multiplier1Core2) {
  if (flagRatio) {
    displayRatio(multiplier1Core2, ROW1_RATIO);

    lastMultiplier2Core2 = multiplier2Core2;
    lastMultiplier1Core2 = multiplier1Core2;
    flagRatio = false;
  }
  if (flagRatio2) {
    displayRatio(multiplier2Core2, ROW2_RATIO);
    flagRatio2 = false;
  }
  //}
  //}
  // if (isFreeRunning2Core2) {
  // if (lastPeriodFreeRunning2Core2 != periodFreeRunning2Core2) {

  // }
  //} else {
  // if (lastMultiplier2Core2 != multiplier2Core2) {

  //}
  //}
  // delay(10);
}