// branch lfo24
// 8/10, sacamos reset, fix sync in, taps 4
// 22/10, volvimos a poner reset, fix double trigger, clock out es thru de clock in, fix auto clock ext
// 23/10 agrega midi clock (usb por ahora), deteccion automatica midi clock, clock ext, int. Emprolijamos algunas cosas
// como se manda el clock externo, sumamos midienabled, etc.
#include <Adafruit_GFX.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <ArduinoTapTempo.h>
#include <Bounce2.h>
#include <EEPROM.h>
#include <Fonts/Font4x7Fixed.h>
/*#include <Fonts/Font4x5Fixed.h>
#include <Fonts/Font4x5Fixed.h>
#include <Fonts/Font4x7Fixed.h>
#include <Fonts/Font5x5Fixed.h>
#include <Fonts/Font5x7Fixed.h>
#include <Fonts/Font5x7FixedMono.h>*/
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans7pt7b.h>
#include <Fonts/FreeSans8pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Lfo.h>
#include <MIDI.h>
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

#define MIDI_RX 9
#define MIDI_TX 8
#define COMPENSATION 1       // 0.9985   // DEJAR EN 1!!!!!
#define LED_REFRESH 33       // 1000/33 30fps
#define LED_BRIGHTNESS 0.05  // 0. a 1.
#define LED_BRIGHTNESS_ENC 1
#define LED_CLOCK_IN 5
//#define LED_LFO1 4
//#define LED_LFO2 1
//#define LED_ENCODER1 0
//#define LED_ENCODER2 2
#define LED_LFO1 3
#define LED_LFO2 2
#define LED_ENCODER1 1
#define LED_ENCODER2 0
#define LED_PIN 16
#define NUM_LEDS 6
#define BUILTIN_LED 25
#define ALARM_NUM 1
#define ALARM_IRQ TIMER_IRQ_1
#define POT1_PIN 27
#define POT2_PIN 28
#define BTN1_PIN 15
#define BTN2_PIN 14
#define TAP_PIN 12
#define CLICK_PIN 13
#define TAP_JACK 22
#define ENC_PINA 3
#define ENC_PINB 2
#define LFO1_PIN 21  // era 20
#define LFO2_PIN 20  // era 21
#define SYNC1_OUT_PIN 17
#define SYNC2_OUT_PIN 18
#define CLOCK_OUT_PIN 19
#define CLOCK_IN_PIN 26
#define MAX_BPM 340
#define MIN_BPM 40
#define ROW1_WAVE 64  // coordenadas Y para el dibujo de formas de onda
#define ROW2_WAVE 88
#define ROW1_RATIO 68
#define ROW2_RATIO 92
#define TRIGGER_DUTY 200  // 200 * 10khz = 20mS
#define POS 1
#define NEG 0
#define TIME_OUT_CLOCK_IN 5000
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
Adafruit_NeoPixel leds(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

Adafruit_USBD_MIDI usb_midi;
// MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);
MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, MIDI);  // esta es para din
ArduinoTapTempo tap;
Bounce wave1 = Bounce();
Bounce wave2 = Bounce();
Bounce tapButton = Bounce();
Bounce tapExt = Bounce();
Bounce clockIn = Bounce();
Bounce clickEncoder = Bounce();
PioEncoder encoder(2);
elapsedMillis btnWaveTime1;
elapsedMillis btnWaveTime2;
elapsedMillis tapSaveTime;
elapsedMillis ledsFps;
elapsedMillis timeOutCounter;  // cuanto tiempo estuvo HIGH
elapsedMillis tapLed;          // duty
// prototypes
void updateButtons();
void updatePots();
void updateEncoder();
void tapTempo();
void updateBpm();
void updateLfoLeds();
void updateTempoLed();
void resetAll();
void clockInPullUp();
float msToBpm(float period);
float bpmToMs(float bpm);
byte readEEPROM(byte address);
void syncPulse();
void sendMidiClock();
float calculateMedian(float newReading);
void sort(float* start, float* end);
void setupMenuInit();
static void alarm_in_us_arm(uint32_t delay_us);
static void alarm_irq(void);
static void alarm_in_us(uint32_t delay_us);

const float multipliers[] = {0.25, 0.5, 0.66666, 1., 1.5, 2, 3., 4., 8., 16., 32.};  // era 0.25, 0.5, 0.66666, 1., 1.5, 2, 3., 4.
const uint8_t numMultipliers = (sizeof(multipliers) / sizeof(byte*));
const uint8_t multipliersSync[numMultipliers] = {4, 2, 6, 1, 2, 1, 1, 1, 1, 1, 1};  // era 4,2,6,1,2,1,1,1
const char* multipliersOled[numMultipliers] = {"1/16", "1/8 ", "1/4t", "1/4 ", "1/4*", "1/2 ", "1/2*", "1   ", "2   ", "4   ", "8   "};
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
volatile float pulsePeriodMsClock;
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
float periodFreeRunning1Core2;
float periodFreeRunning2Core2;
bool btnWaveHold1;
bool btnWaveHold2;
bool isFreeRunning1 = 0;
bool isFreeRunning2 = 0;
bool isFreeRunning1Core2 = 0;
bool isFreeRunning2Core2 = 0;
float periodFreeRunning1;
float periodFreeRunning2;
bool setupState = 0;
bool bootState = 0;
/// para display
bool flagBpm;
bool flagRatio;
bool flagRatio2;
bool flagWave;
bool flagWave2;
bool flagFreq;
bool flagFreq2;
volatile bool midiClockConnected;
bool lastMidiClockConnected;
volatile int timeOutCounterMidi;
// int MIN_TAPS = 2;
float calculateMedian(float newReading) {
  const int bufferSize = 8;
  static float readings[bufferSize] = {0};  // Array to store readings
  static int currentIndex = 0;              // Index to store the next reading
  static int totalReadings = 0;             // Total number of readings received so far (max 8)

  // Store the new reading at the current position
  readings[currentIndex] = newReading;
  currentIndex = (currentIndex + 1) % bufferSize;

  // Increase the total reading count until it reaches the buffer size
  if (totalReadings < bufferSize) {
    totalReadings++;
  }

  // Sort a copy of the readings array to calculate the median
  float sortedReadings[bufferSize];
  memcpy(sortedReadings, readings, sizeof(readings));    // Copy readings to a new array
  sort(sortedReadings, sortedReadings + totalReadings);  // Sort the copied array

  // Return the median value
  if (totalReadings % 2 == 1) {
    return sortedReadings[totalReadings / 2];  // If odd, return the middle value
  } else {
    return (sortedReadings[(totalReadings / 2) - 1] + sortedReadings[totalReadings / 2]) / 2.0;  // If even, return the average of the two middle values
  }
}

void sort(float* start, float* end) {
  // Simple bubble sort (you could replace this with a more efficient sorting algorithm if needed)
  for (float* i = start; i != end; ++i) {
    for (float* j = i + 1; j != end; ++j) {
      if (*i > *j) {
        float temp = *i;
        *i = *j;
        *j = temp;
      }
    }
  }
}

void onClock() {
  // Serial.println("got clock");
  // pulseConnected = true;
  midiClockConnected = true;
  timeOutCounterMidi = 0;
  // lfo.resetPhaseMaster();
  // lfo.resetPhaseMaster();  // reseteamos el master clock interno solo con los pulsos
  //  antes el reset era igual que los lfos, entonces se triggeaba el clock out con cada syncout
  // counterTimeOut = 0;     // este es el watchdog, resetea a 0 en todos los pulsos
  if (!isFreeRunning1) {  // el sync lo activa solo si no esta en free
    lfo.enableSync(0);
    lfo.enableMidi(0);
  }  // mover esto a otro lado...
  if (!isFreeRunning2) {
    lfo.enableSync(1);
    lfo.enableMidi(1);
  }
  // MIDI.sendClock();
  MIDI.sendRealTime(midi::Clock);
  // if(ratioLfo1 == 1){
  //   digitalWrite(SYNC1_OUT_PIN, HIGH); //thru
  //   counterDivTicksLfo1 = 0;

  //}

  if (pulseCounter % 2 == 0) {  // always
    counterTicksPulse = 0;
  } else if (pulseCounter % 2 == 1) {  // always
                                       // pulsePeriod = counter * 0.5; //creo que lo calculamos fuera del timer, aca y en ratio

    pulseTicksLfo1 = counterTicksPulse * ratioLfo1 * 24;
    pulsePeriodMsClock = counterTicksPulse * SAMPLE_RATE_MS * 24;
    pulsePeriodMsLfo1 = (counterTicksPulse * SAMPLE_RATE_MS * 24);  //* 1.00120144;
    pulseTicksLfo2 = counterTicksPulse * ratioLfo2 * 24;
    // pulsePeriodMsLfo2 = counterTicksPulse * SAMPLE_RATE_MS; //en verdad solo necesito el pulseperiod de uno solo
    if (!isFreeRunning1) {  // lo mismo aca con el free running
      lfo.setPeriodMs(0, pulsePeriodMsLfo1);
    }
    if (!isFreeRunning2) {
      lfo.setPeriodMs(1, pulsePeriodMsLfo1);
    }

    // con el nuevo codigo x24 en vez de resetear cada lfo, reseteamos el master, xq los slaves derivan de el
    // Serial.println(pulsePeriodMsLfo1);

    gotNewPulse = true;
  }
  // lfo.triggerReset();
  if (pulseCounter % 24 == 0) {  // negra. El clock out es thru tanto en midi como clock ext.
    lfo.clockOut(HIGH);          // la polaridad la seteamos en la clase comentado 032025
    lfo.clockFromExt();

    // Serial.print(calculateMedian(pulsePeriodMsClock));
    // Serial.print("  ");
    // Serial.println(pulsePeriodMsLfo1);
    lfo.resetPhaseMaster();

    if (pulsePeriodMsClock > 0) {  // para que no tire inf si no paso nada
      lfo.setPeriodMsClock(pulsePeriodMsClock);
      bpm = msToBpm(pulsePeriodMsClock);  // para que si se saca la sincro recuerde el bpm... MEJORAR EMPROLIJAR
      tap.setBPM(bpm);
    }
  }
  // Serial.println(pulsePeriodMsClock * 24);

  if (pulseCounter % int(ceil(ratioLfo1 * 24)) == 0) {  // * 24 ppqn
    // counterTicksPulse = 0;
    //  digitalWrite(17, HIGH);
    counterDivTicksLfo1 = 0;
    Serial.println("LFO1");
  }
  if (pulseCounter % int(ceil(ratioLfo2 * 24)) == 0) {
    // counterTicksPulse = 0;
    //  digitalWrite(17, HIGH);
    Serial.println("LFO2");
    counterDivTicksLfo2 = 0;
  }
  if (pulseCounter % 24 == 0) {
    // clock thru.
  }
  if (pulseCounter % 24 == 2) {  // 2 pulsos de duty clock out
    lfo.clockOut(LOW);
  }

  pulseCounter++;
}
void onStart() {
  midiClockConnected = true;
  pulseConnected = false;
  pulseCounter = 0;
  lfo.clockOut(HIGH);
  lfo.clockFromExt();
  counterTicksPulse = 0;
  counterDivTicksLfo1 = 0;
  counterDivTicksLfo2 = 0;
  counterDivTicksLfo1 = 0;
  counterDivTicksLfo2 = 0;
  lfo.resetPhaseMaster();
  lfo.resetPhase(0);
  lfo.resetPhase(1);
  // MIDI.sendStart();
  MIDI.sendRealTime(midi::Start);
}
byte TAPS;
bool LFO1_PULSE;
bool LFO2_PULSE;
byte LFO1_RANGE;
byte LFO2_RANGE;
byte MASTER_PULSE;
byte MASTER_PPQN;
byte ENC_DIR;

const byte TAPS_ADDR = 0;
const byte LFO1_RANGE_ADDR = 1;
const byte LFO2_RANGE_ADDR = 2;
const byte LFO1_PULSE_ADDR = 3;
const byte LFO2_PULSE_ADDR = 4;
const byte MASTER_PULSE_ADDR = 5;
const byte MASTER_PPQN_ADDR = 6;
const byte ENC_DIR_ADDR = 7;
byte defaultValues[8] = {2, 1, 1, 1, 1, 1, 0, 0};
/*EEPROM STRUCTURE
byte 0 = TAPS
byte 1 = LFO1 RANGE
byte 2 = LFO2 RANGE
byte 3 = LFO1 PULSE
byte 4 = LFO2 PULSE
byte 5 = MASTER PULSE
byte 6 = MASTER PPQN*/
byte readEEPROM(byte address) {
  byte value = EEPROM.read(address);
  if (value != 255) {
    return value;
  } else {
    return defaultValues[address];
  }
}
float freeRunningRange[3][2] = {
    {100, 100000.},  // SLOW 200ms - 10000ms
    {100., 10000.},  // MID 20ms - 1000mS
    {4., 1000.}      // FAST 4ms - 20ms
};
byte ppqn[3]{1, 2, 4};
void setup() {
  Serial.begin(115200);
  Serial2.setRX(MIDI_RX);
  Serial2.setTX(MIDI_TX);
  MIDI.begin();
  MIDI.turnThruOff();
  MIDI.setHandleClock(onClock);
  MIDI.setHandleStart(onStart);
  EEPROM.begin(256);
  TAPS = readEEPROM(TAPS_ADDR);
  LFO1_RANGE = readEEPROM(LFO1_RANGE_ADDR);
  LFO2_RANGE = readEEPROM(LFO2_RANGE_ADDR);
  LFO1_PULSE = readEEPROM(LFO1_PULSE_ADDR);
  LFO2_PULSE = readEEPROM(LFO2_PULSE_ADDR);
  MASTER_PULSE = readEEPROM(MASTER_PULSE_ADDR);
  MASTER_PPQN = readEEPROM(MASTER_PPQN_ADDR);
  ENC_DIR = readEEPROM(ENC_DIR_ADDR);
  encoder.begin();
  if (ENC_DIR == 1) {
    encoder.flip();
  }
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
  pinMode(CLOCK_IN_PIN, INPUT_PULLUP);
  pinMode(7, INPUT);  // dummy pin GP7 a GP9 RX
  // pinMode(9, INPUT);
  wave1.attach(BTN1_PIN, INPUT_PULLUP);
  wave2.attach(BTN2_PIN, INPUT_PULLUP);
  wave1.interval(25);
  wave2.interval(25);
  tapButton.attach(TAP_PIN, INPUT_PULLUP);
  tapButton.interval(25);
  clickEncoder.attach(CLICK_PIN, INPUT_PULLUP);
  clickEncoder.interval(25);
  tapExt.attach(TAP_JACK, INPUT_PULLUP);
  tapExt.interval(25);
  clockIn.attach(CLOCK_IN_PIN, INPUT_PULLUP);
  clockIn.interval(100);
  tapButton.update();
  lfo.setTriggerPeriod(0, TRIGGER_PERIOD);
  lfo.setTriggerPeriod(1, TRIGGER_PERIOD);
  lfo.setPeriodMs(0, 500);
  lfo.setPeriodMs(1, 500);
  lfo.setPeriodMsClock(500);
  lfo.setRatio(0, 1);
  lfo.setRatio(1, 1);
  lfo.setTriggerPolarity(0, LFO1_PULSE);
  lfo.setTriggerPolarity(1, LFO2_PULSE);
  lfo.setClockPolarity(MASTER_PULSE);
  lfo.setClockPPQN(ppqn[MASTER_PPQN]);
  clickEncoder.update();
  delay(100);
  if (clickEncoder.read() == 0) {
    // bootState = true;
  }

  if (tapButton.read() == 0) {
    setupState = true;
  }
  if (setupState == false && bootState == false) {
    alarm_in_us(ALARM_PERIOD);

    attachInterrupt(digitalPinToInterrupt(CLOCK_IN_PIN), syncPulse, RISING);
  }

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
  updateButtons();
  updatePots();
  lfo.resetPhase(0);
  lfo.resetPhase(1);
  lfo.resetPhaseMaster();
  tap.setTotalTapValues(8);
  // displayBpm(120.0);
}
void syncPulse() {
  if (!midiClockConnected) {  // midi clock tiene prioridad
    pulseConnected = true;
    timeOutCounter = 0;
    // lfo.resetPhaseMaster();
    // lfo.resetPhaseMaster();  // reseteamos el master clock interno solo con los pulsos
    //  antes el reset era igual que los lfos, entonces se triggeaba el clock out con cada syncout
    // counterTimeOut = 0;     // este es el watchdog, resetea a 0 en todos los pulsos
    if (!isFreeRunning1) {  // el sync lo activa solo si no esta en free
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
        pulsePeriodMsClock = calculateMedian(counterTicksPulse * SAMPLE_RATE_MS);  // new 30/10
        pulsePeriodMsLfo1 = (counterTicksPulse * SAMPLE_RATE_MS);                  //* 1.00120144;
        pulseTicksLfo2 = counterTicksPulse * ratioLfo2;
        // pulsePeriodMsLfo2 = counterTicksPulse * SAMPLE_RATE_MS; //en verdad solo necesito el pulseperiod de uno solo
        if (!isFreeRunning1) {  // lo mismo aca con el free running
          // lfo.setPeriodMs(0, pulsePeriodMsLfo1);
          lfo.setPeriodMs(0, pulsePeriodMsClock);
        }
        if (!isFreeRunning2) {
          // lfo.setPeriodMs(1, pulsePeriodMsLfo1);
          lfo.setPeriodMs(1, pulsePeriodMsClock);
        }

        // con el nuevo codigo x24 en vez de resetear cada lfo, reseteamos el master, xq los slaves derivan de el
        // Serial.println(pulsePeriodMsLfo1);

        gotNewPulse = true;
      }
    }
    // lfo.triggerReset();
    lfo.clockFromExt();
    lfo.resetPhaseMaster();

    Serial.println(pulsePeriodMsClock);
    if (pulsePeriodMsClock > 0) {  // para que no tire inf si no paso nada
      lfo.setPeriodMsClock(pulsePeriodMsClock);
      bpm = msToBpm(pulsePeriodMsClock);
      tap.setBPM(bpm);
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

  if (pulseConnected || midiClockConnected) {
    if (pulseConnected) {
      bool clockInState = digitalRead(CLOCK_IN_PIN);
      lfo.clockOut(clockInState);  // clock thru.
    }
    if (counterDivTicksLfo1 % pulseTicksLfo1 == 0) {  // LFO1
      if (newTriggerLfo1) {
        // digitalWrite(SYNC1_OUT_PIN, HIGH);
        if (!isFreeRunning1) {
          lfo.resetPhase(0);  // habia sacado esto pero creo que lo dejo para el lfo x24
        }
        // lfo.resetPhase(1);
        newTriggerLfo1 = false;
      }
    } else if (counterDivTicksLfo1 % pulseTicksLfo1 == TRIGGER_DUTY) {
      // digitalWrite(SYNC1_OUT_PIN, LOW);
      newTriggerLfo1 = true;
    }
    if (counterDivTicksLfo2 % pulseTicksLfo2 == 0) {  // LFO2
      if (newTriggerLfo2) {
        // digitalWrite(SYNC1_OUT_PIN, HIGH);
        if (!isFreeRunning2) {
          // lfo.resetPhaseMaster();

          // Serial.println("reset");
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
    // counterTimeOut++;
    if (midiClockConnected) {
      timeOutCounterMidi++;
      if (timeOutCounterMidi > 5000) {
        midiClockConnected = false;
      }
    }
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
  // Serial.println(clickEncoder.read());
  MIDI.read();
  updatePots();
  updateButtons();
  updateEncoder();
  tapTempo();

  if (lfo.hasMidiClock()) {
    // Serial.println("midi");
    // MIDI.sendClock();
    MIDI.sendRealTime(midi::Clock);
  }

  clockInPullUp();
  if (gotNewPulse) {
    gotNewPulse = false;
  }

  if (lastGotNewPulse != gotNewPulse) {
    updateBpm();

    lastGotNewPulse = gotNewPulse;
  }
  /*if (pulseConnected && counterTimeOut > timeOut) { //comentado 8/10
    resetAll();
    pulseConnected = false;
    Serial.println("pulseDisconnected");
  }*/
  if (lastPulseConnected != pulseConnected) {
    if (pulseConnected) {  // EMPROLIJAR ESTO.
      // displayBpm(0);  // 0 == ext clock
      bpmCore2 = 0;
      flagBpm = true;
      // newBpm = true;
      Serial.println("pulseConnected");
      lfo.setExtClock(true);
    } else {
      timeOutCounter = 0;
      // delay(100);
      lfo.clockOut(LOW);  // polaridad en clase, no me ta funcionando igual...
      lfo.setExtClock(false);
      if (!midiClockConnected) {  // que vuelva a resetear todo si no hay midi clock
        resetAll();
      }
    }
    Serial.println(pulseConnected);
    lastPulseConnected = pulseConnected;
  }
  if (lastMidiClockConnected != midiClockConnected) {
    if (midiClockConnected) {  // EMPROLIJAR ESTO.
      // displayBpm(0);  // 0 == ext clock
      bpmCore2 = -1;  // midi
      flagBpm = true;
      // newBpm = true;
      Serial.println("midiClockConnected");
      lfo.setExtClock(true);
      lfo.enableMidi(0);
      lfo.enableMidi(1);
    } else {
      timeOutCounterMidi = 0;

      lfo.setExtClock(false);
      lfo.disableMidi(0);
      lfo.disableMidi(1);
      Serial.println("midiClockDisconnected");
      resetAll();
    }
    Serial.println(midiClockConnected);
    lastMidiClockConnected = midiClockConnected;
  }
}

void clockInPullUp() {
  clockIn.update();
  static bool flagTimeOut;
  /*if (clockIn.rose() || clockIn.fell()) {
    timeOutCounter = 0;
    flagTimeOut = true;
    // Serial.println("se conecto");
  }
  // if (clockIn.read() == HIGH && timeOutCounter > TIME_OUT_CLOCK_IN && flagTimeOut) {
  if (timeOutCounter > TIME_OUT_CLOCK_IN && flagTimeOut) {
    flagTimeOut = false;

     pulseConnected = false;
    // pulsePeriodMsClock
    resetAll();
    //Serial.println("se desconecto");
  }*/
  if (clockIn.read() == HIGH || clockIn.read() == LOW) {
    // Serial.println(timeOutCounter);
    if (timeOutCounter > TIME_OUT_CLOCK_IN) {
      // flagTimeOut = false;
      // lfo.clockOut(LOW);
      pulseConnected = false;
      // resetAll();
    }
  }
  // Serial.print(flagTimeOut);
  // Serial.print(" ");
  // Serial.println(timeOutCounter);
}
void resetAll() {
  lfo.disableSync(0);
  lfo.disableSync(1);
  lfo.resetPhase(0);
  lfo.resetPhase(1);
  lfo.resetPhaseMaster();
  updateBpm();
  leds.setPixelColor(LED_CLOCK_IN, leds.Color(0, 0, 0));  // apaga led si no hay clock in
  leds.show();
}
void updatePots() {
  potMult1.update();
  potMult2.update();
  static int lastMultiplier1 = -1;
  static int lastMultiplier2 = -1;

  multiplier1 = map(potMult1.getValue(), 0, 1023, 0, numMultipliers);
  // multiplier1 = map(potMult1.getValue(), 1023, 0, 0, numMultipliers);  // potes estan invertidos en el hard
  multiplier2 = map(potMult2.getValue(), 0, 1023, 0, numMultipliers);
  // multiplier2 = map(potMult2.getValue(), 1023, 0, 0, numMultipliers);
  // Serial.println(potMult1.getValue());
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
  // periodFreeRunning1 = fscale(potMult1.getValue(), 0, 1023, 20, 1000, -5);
  periodFreeRunning1 = fscale(potMult1.getValue(), 0, 1023, freeRunningRange[LFO1_RANGE][0], freeRunningRange[LFO1_RANGE][1], -5);
  // periodFreeRunning1 = fscale(potMult1.getValue(), 1023, 3, 1000, 100, 7);
  if (isFreeRunning1) {
    if (potMult1.hasChanged()) {
      lfo.setPeriodMs(0, periodFreeRunning1);
      periodFreeRunning1Core2 = periodFreeRunning1;
      Serial.println(periodFreeRunning1Core2);
      flagFreq = true;
      // Serial.print(potMult1.getValue());
      // Serial.print("   ");
      // Serial.println(periodFreeRunning1);
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
  // bla
  // periodFreeRunning2 = fscale(potMult2.getValue(), 0, 1023, 20, 1000, -5);
  periodFreeRunning2 = fscale(potMult2.getValue(), 0, 1023, freeRunningRange[LFO2_RANGE][0], freeRunningRange[LFO2_RANGE][1], -5);
  // periodFreeRunning2 = fscale(potMult2.getValue(), 1023, 3, 1000, 100, 7);
  if (isFreeRunning2) {
    if (potMult2.hasChanged()) {
      // periodFreeRunning2Core2 = periodFreeRunning2;
      periodFreeRunning2Core2 = periodFreeRunning2;
      flagFreq2 = true;
      Serial.print(potMult2.getValue());
      Serial.print("  ");
      Serial.println(periodFreeRunning2);
      lfo.setPeriodMs(1, periodFreeRunning2);
    }
  }
}

bool setupButton1Flag;
bool setupButton2Flag;
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
    setupButton1Flag = true;
  }
  if (wave1.read() == LOW && btnWaveTime1 >= 1000 && btnWaveHold1) {
    isFreeRunning1 = !isFreeRunning1;
    isFreeRunning1Core2 = isFreeRunning1;
    if (isFreeRunning1) {
      lfo.disableSync(0);
      // updatePots();
      Serial.println("free running1");
      periodFreeRunning1Core2 = periodFreeRunning1;
      flagFreq = true;
      lfo.turnFreeRunning(0, isFreeRunning1);
      lfo.setPeriodMs(0, periodFreeRunning1);
      lfo.setRatio(0, 1);

    } else {
      multiplier1 = map(potMult1.getValue(), 0, 1023, 0, numMultipliers);
      ratioLfo1 = multipliers[multiplier1];
      multiplier1Core2 = multiplier1;
      multiplierSyncLfo1 = multipliersSync[multiplier1];
      Serial.println("wave1");
      flagRatio = true;
      lfo.setPeriodMs(0, periodMs * COMPENSATION);
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
    setupButton2Flag = true;
  }
  if (wave2.read() == LOW && btnWaveTime2 >= 1000 && btnWaveHold2) {
    isFreeRunning2 = !isFreeRunning2;
    isFreeRunning2Core2 = isFreeRunning2;
    lfo.disableSync(1);
    if (isFreeRunning2) {
      periodFreeRunning2Core2 = periodFreeRunning2;
      flagFreq2 = true;
      Serial.println("free running2");
      lfo.turnFreeRunning(1, isFreeRunning2);
      lfo.setPeriodMs(1, periodFreeRunning2);
      lfo.setRatio(1, 1);  // cuando es free running el ratio tiene que ser 1.

    } else {
      multiplier2 = map(potMult2.getValue(), 0, 1023, 0, numMultipliers);
      ratioLfo2 = multipliers[multiplier2];
      multiplierSyncLfo2 = multipliersSync[multiplier2];
      multiplier2Core2 = multiplier2;
      flagRatio2 = true;
      Serial.println("wave2");
      lfo.setPeriodMs(1, periodMs * COMPENSATION);
      lfo.setRatio(1, ratioLfo2);
      lfo.turnFreeRunning(1, isFreeRunning2);
    }
    btnWaveHold2 = false;
  }
}

bool tapSaveFlag;

void tapTempo() {
  static bool flagTapLed;
  static int tapCount;
  tapButton.update();
  if (!pulseConnected && !midiClockConnected) {
    // tapButton.update();
    tapState = tapButton.read() ^ tapExt.read();  // deberia ser OR pero como es invertido usamos XOR.
    tap.update(tapState);

    if (tapButton.fell() || tapExt.fell()) {
      // if reset...
      lfo.triggerReset();
      lfo.resetPhase(0);  // esto comentado para sacar el reset
      lfo.resetPhase(1);
      lfo.resetPhaseMaster();

      // leds.setPixelColor(LED_CLOCK_IN, leds.Color(255 * LED_BRIGHTNESS, 0, 255 * LED_BRIGHTNESS));
      tapLed = 0;
      flagTapLed = true;
      tapCount++;
      if (tapCount >= TAPS) {
        periodMs = tap.getBeatLength();
        // tap.update(tapState);
        if (!isFreeRunning1) {
          lfo.setPeriodMs(0, periodMs * COMPENSATION);
        }
        if (!isFreeRunning2) {
          lfo.setPeriodMs(1, periodMs * COMPENSATION);
        }
        lfo.setPeriodMsClock(periodMs * COMPENSATION);
        bpm = msToBpm(periodMs);
        Serial.println(bpm);
        bpmCore2 = bpm;
        flagBpm = true;
      }
      Serial.println(tapCount);

      // displayBpm(bpm);
    }
    if (tapLed > 100 && flagTapLed) {
      // leds.setPixelColor(LED_CLOCK_IN, leds.Color(0, 0, 0));
      // leds.show();
      tapLed = 0;
      flagTapLed = false;
    }
  }
  if (tap.isChainActive() == false) {
    tapCount = 0;
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
  if (!pulseConnected && !midiClockConnected) {
    periodMs = bpmToMs(bpm);
    if (!isFreeRunning1) {  // si no me modifica la freq de los lfo en free running
      lfo.setPeriodMs(0, periodMs * COMPENSATION);
    }
    if (!isFreeRunning2) {
      lfo.setPeriodMs(1, periodMs * COMPENSATION);
    }
    lfo.setPeriodMsClock(periodMs * COMPENSATION);
    // displayBpm(bpm);
    bpmCore2 = bpm;
    flagBpm = true;
    // pushCore2 = true;
    tap.setBPM(bpm);  // para que el tap no salte a cualquier valor, que se vaya ajustando
  }
}
bool setupEncoderFlag;
int setupEncoderResult;
int encoderResult;
bool clickEncoderCore2;
void updateEncoder() {
  static int encoderValue;
  static int lastEncoderValue;
  clickEncoder.update();
  // static int encoderResult;
  encoderValue = encoder.getCount() >> 2;  // dividido 4
  if (encoderValue != lastEncoderValue) {
    if (encoderValue > lastEncoderValue) {
      encoderResult = 1;

      // Serial.println("sube");
    } else {
      // Serial.println("baja");
      encoderResult = -1;
    }
    setupEncoderFlag = true;
    if (!pulseConnected || !midiClockConnected) {  // que lo refresque si no hay clock externo
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
  clickEncoderCore2 = clickEncoder.read();
}

void updateLfoLeds() {
  uint8_t ledLfo1 = lfo.getLfoValues(0);
  uint8_t ledLfo2 = lfo.getLfoValues(1);
  if (ledsFps > LED_REFRESH) {
    ledsFps = 0;
    // leds.setPixelColor(5, leds.Color(random(256), 100, 30));
    leds.setPixelColor(LED_LFO1, leds.Color(ledLfo1 * LED_BRIGHTNESS, 0, 0));
    leds.setPixelColor(LED_LFO2, leds.Color(ledLfo2 * LED_BRIGHTNESS, 0, 0));
    // leds.setPixelColor(LED_LFO1, leds.Color(ledLfo1 * LED_BRIGHTNESS, ledLfo2 * 0.1, ledLfo2 * 0.2));
    // leds.setPixelColor(LED_LFO2, leds.Color(ledLfo2 * LED_BRIGHTNESS, ledLfo1 * 0.1, ledLfo1 * 0.2));
    leds.show();
  }
}

void updateTempoLed() {
  bool ledTempo = lfo.getClockOut();
  static bool ledTempoLast;
  if (ledTempo != ledTempoLast) {
    if (pulseConnected) {  // esta en sync in parpadea leds encoder y led sync in
      leds.setPixelColor(LED_CLOCK_IN, leds.Color(0, 255 * ledTempo * LED_BRIGHTNESS, 0));
      leds.setPixelColor(LED_ENCODER1, leds.Color(0, 255 * ledTempo * LED_BRIGHTNESS_ENC, 0));
      leds.setPixelColor(LED_ENCODER2, leds.Color(0, 255 * ledTempo * LED_BRIGHTNESS_ENC, 0));

    } else if (midiClockConnected) {
      // leds.setPixelColor(LED_CLOCK_IN, leds.Color(255 * ledTempo * LED_BRIGHTNESS_ENC, 255 * ledTempo * LED_BRIGHTNESS, 0));
      leds.setPixelColor(LED_ENCODER1, leds.Color(255 * ledTempo * LED_BRIGHTNESS_ENC, 255 * ledTempo * LED_BRIGHTNESS_ENC, 0));
      leds.setPixelColor(LED_ENCODER2, leds.Color(255 * ledTempo * LED_BRIGHTNESS_ENC, 255 * ledTempo * LED_BRIGHTNESS_ENC, 0));

    } else {
      leds.setPixelColor(LED_ENCODER1, leds.Color(0, 0, 255 * ledTempo * LED_BRIGHTNESS_ENC));
      leds.setPixelColor(LED_ENCODER2, leds.Color(0, 0, 255 * ledTempo * LED_BRIGHTNESS_ENC));
    }
    leds.show();
    ledTempoLast = ledTempo;
    // Serial.println(ledTempo);
  }
}
//////CORE 2 PARA OLED Y NEOPIXEL
void displayBpm(float bpm) {
  if (!setupState && !bootState) {
    oled.setCursor(0, 8);
    oled.fillRect(0, 0, 63, 22, BLACK);
    oled.setTextSize(2);
    if (bpm != 0 && bpm != -1) {  // si no es bpm 0 es bpm, si no es EXT.
      oled.print(bpm, 1);
    } else if (bpm == 0) {
      oled.print("EXT ");
    } else if (bpm == -1) {
      oled.print("MIDI");
    }
    oled.display();
  }
  // flagBpm = false;
}

void displayRatio(int multiplier, byte row) {
  if (!setupState && !bootState) {
    oled.setTextSize(1);
    oled.setCursor(36, row);
    oled.print(multipliersOled[multiplier]);
    oled.display();
  }
  // flagRatio = false;
}

void displayFreq(float period, byte row) {
  if (!setupState && !bootState) {
    oled.setTextSize(1);
    oled.setCursor(36, row);
    oled.fillRect(35, row - 1, 35, 14, BLACK);
    float freq = 1000. / period;
    if (freq <= 9.98) {
      oled.print(freq, 2);
    }
    if (freq >= 9.99 && freq < 100.) {
      oled.print(freq, 1);
    } else if (freq > 99.9) {
      oled.print(freq, 0);
    }
    /*if (period < 1000) {
      oled.print(freq, 1);
    } else {
      oled.print(freq, 2);
    }*/
    oled.display();
  }
}

void displayWave(byte wave, byte row) {
  if (!setupState && !bootState) {
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
  // flagWave = false;
}

void updatePage(bool nextPage);
void updateValue(int delta);
void displayMenu();
struct PageConfig {
  int minVal;            // Minimum value for this page
  int maxVal;            // Maximum value for this page
  const char* pageName;  // Display name of the page            // Rectangle height
  int currentValue;      // Last value set for this page
  int valueX;
  int valueY;
  int titleX;
  int titleY;
};
PageConfig pages[] = {
    {2, 8, "TAPS", 0, 24, 90, 0, 45},  // Starting at minVal
    {0, 2, "LFO 1", 0, 0, 90, 0, 45},  // espacios ajustados para cambio de linea automatico
    {0, 2, "LFO 2", 0, 0, 90, 0, 45},
    {0, 1, "LFO 1", 0, 0, 90, 0, 45},
    {0, 1, "LFO 2", 0, 0, 90, 0, 45},
    {0, 1, "MASTER", 0, 0, 90, 0, 45},
    {0, 2, "MASTER", 0, 0, 90, 0, 45},
    {0, 1, "ENC", 0, 0, 90, 0, 45}};
const char* lfoRanges[3]{"SLOW", "MID", "FAST"};
const char* pulsePolarity[2]{"NEG", "POS"};
const char* encoderDir[2]{"<<", ">>"};
int ppqnValues[3]{1, 2, 4};
const int totalPages = sizeof(pages) / sizeof(pages[0]);
int currentPage = -1;  // Start with the first page
// int currentValue = 0;
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
  // oled.setCursor(32, 32);
  // oled.print("miditest");
  if (setupState && !bootState) {
    oled.setFont(&FreeSans7pt7b);
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    // pages[0].currentValue = EEPROM.read(0);
    pages[0].currentValue = TAPS;
    pages[1].currentValue = LFO1_RANGE;
    pages[2].currentValue = LFO2_RANGE;
    pages[3].currentValue = LFO1_PULSE;
    pages[4].currentValue = LFO2_PULSE;
    pages[5].currentValue = MASTER_PULSE;
    pages[6].currentValue = MASTER_PPQN;
    pages[7].currentValue = ENC_DIR;
    setupMenuInit();
    updatePage(1);
    updateValue(0);
  }
  if (!setupState && bootState) {
    oled.setFont(&FreeSans7pt7b);
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print("BOOOOOT");
    oled.display();
  }
}
void setupMenuInit() {
  oled.clearDisplay();
  oled.setCursor(0, 14);
  oled.setTextSize(1);
  // oled.setTextColor(SSD1306_WHITE);
  oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  oled.print("SETUP");
  // oled.setTextSize(1);
  /*oled.setCursor(0, 30);
  oled.print("Taps");
  oled.setCursor(24, 52);
  // oled.startScrollRight();
  oled.print("4");
  oled.display();*/
}

float lastBpmCore2;
byte lastWaveSelector1Core2;
byte lastWaveSelector2Core2;
int lastMultiplier1Core2;
int lastMultiplier2Core2;
int lastPeriodFreeRunning1Core2;
int lastPeriodFreeRunning2Core2;

void displayMenu() {
  oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

  oled.print(pages[currentPage].currentValue);
  oled.display();
}
void updateValue(int delta) {
  // Modify the value within the current page’s limits
  pages[currentPage].currentValue += delta;
  if (pages[currentPage].currentValue < pages[currentPage].minVal)
    pages[currentPage].currentValue = pages[currentPage].minVal;
  if (pages[currentPage].currentValue > pages[currentPage].maxVal)
    pages[currentPage].currentValue = pages[currentPage].maxVal;
  oled.setFont(&FreeSans9pt7b);
  oled.fillRect(0, pages[currentPage].valueY - 16, 64, 18, SSD1306_BLACK);
  oled.setCursor(pages[currentPage].valueX, pages[currentPage].valueY);

  if (currentPage == 0) {
    oled.print(pages[currentPage].currentValue);
  } else if (currentPage == 1 || currentPage == 2) {
    oled.print(lfoRanges[pages[currentPage].currentValue]);
  } else if (currentPage == 3 || currentPage == 4 || currentPage == 5) {
    oled.print(pulsePolarity[pages[currentPage].currentValue]);
  } else if (currentPage == 6) {
    oled.print(ppqnValues[pages[currentPage].currentValue]);
  } else if (currentPage == 7) {
    oled.print(encoderDir[pages[currentPage].currentValue]);
  }

  /*if (currentPage != 0) {
    oled.print(lfoRanges[pages[currentPage].currentValue]);
  } else {
    oled.print(pages[currentPage].currentValue);
  }*/
  EEPROM.write(currentPage, pages[currentPage].currentValue);

  oled.display();
}
void updatePage(bool nextPage) {
  if (nextPage) {
    currentPage++;  // Move to the next page
  } else {
    currentPage--;  // Move to the previous page
  }
  if (currentPage < 0)
    currentPage = 0;
  if (currentPage >= totalPages)
    currentPage = totalPages - 1;

  oled.setFont(&FreeSans7pt7b);
  oled.setCursor(pages[currentPage].titleX, pages[currentPage].titleY);
  oled.fillRect(pages[currentPage].titleX, pages[currentPage].titleY - 14, 64, 45, SSD1306_BLACK);
  oled.print(pages[currentPage].pageName);
  if (currentPage == 1 || currentPage == 2) {
    oled.setCursor(pages[currentPage].titleX, pages[currentPage].titleY + 15);
    oled.print("RANGE");
  } else if (currentPage == 3 || currentPage == 4 || currentPage == 5) {
    oled.setCursor(pages[currentPage].titleX, pages[currentPage].titleY + 15);
    oled.print("PULSE");
  } else if (currentPage == 6) {
    oled.setCursor(pages[currentPage].titleX, pages[currentPage].titleY + 15);
    oled.print("PPQN");
  } else if (currentPage == 7) {
    oled.setCursor(pages[currentPage].titleX, pages[currentPage].titleY + 15);
    oled.print("DIR");
  }
  oled.display();
}
bool lastClickEncoderCore2;
int tapButtonHold;
bool savingFlag;
void loop1() {
  if (setupState) {
    if (!savingFlag) {
      if (setupEncoderFlag) {
        // value += encoderResult;
        updateValue(encoderResult);  //+1 o 1
        // displayMenu();
        setupEncoderFlag = false;
      }
      if (setupButton1Flag) {
        updatePage(0);   // 0 baja 1
        updateValue(0);  // que muestre el valor que tiene adentro con cambio de page
        setupButton1Flag = false;
      }
      if (setupButton2Flag) {
        updatePage(1);   // 1 sube 1
        updateValue(0);  // que muestre el valor que tiene adentro con cambio de page
        setupButton2Flag = false;
      }
    }
    // Serial.println(clickEncoderCore2);
    // delay(50);
    if (lastClickEncoderCore2 != clickEncoderCore2) {
      if (clickEncoderCore2 == LOW) {
        tapSaveTime = 0;
        tapButtonHold = true;
        Serial.println(clickEncoderCore2);
      }
      if (clickEncoderCore2 == HIGH) {
        oled.fillRect(0, 120, 64, 5, SSD1306_BLACK);
        tapSaveTime = 0;
        tapButtonHold = false;
        savingFlag = false;
        oled.display();
      }
      lastClickEncoderCore2 = clickEncoderCore2;
    }
    if (clickEncoderCore2 == LOW && tapSaveTime < 2000 && tapButtonHold) {
      int progressBar = map(tapSaveTime, 0, 2000, 0, 64);
      oled.fillRect(0, 120, progressBar, 5, SSD1306_WHITE);
      oled.display();
      savingFlag = true;
    }

    else if (clickEncoderCore2 == LOW && tapSaveTime >= 2000 && tapButtonHold) {
      tapSaveTime = 0;
      tapButtonHold = false;

      Serial.println(pages[0].currentValue);
      // EEPROM.write(0, pages[0].currentValue);
      EEPROM.commit();
      rp2040.reboot();
      Serial.println("save and reset");
    }

    if (tapSaveFlag) {
    }

  }

  else if (!setupState) {
    updateLfoLeds();
    updateTempoLed();
    // uint32_t counterTicksPulseCore = rp2040.fifo.pop();
    // Serial.println(counterTicksPulseCore);
    // delay(20);
    // if (lastBpmCore2 != bpmCore2) {
    if (flagBpm) {
      displayBpm(bpmCore2);
      // Serial.println(flagBpm);
      //  lastBpmCore2 = bpmCore2;
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