#ifndef Lfo_h
#define Lfo_h
#include <Arduino.h>

#include "pico/stdlib.h"
#include "wavetables.h"

class Lfo {
 public:
  Lfo(volatile uint8_t lfoPin1, volatile uint8_t lfoPin2, uint32_t sampleRate, uint32_t tableSize, uint16_t range, uint8_t syncPin1, uint8_t syncPin2);
  void setFreqHz(uint8_t lfoNum, float freq);
  void update();
  void setPeriodMs(uint8_t lfoNum, float period);
  void setPeriodMsClock(float period);  // el clock out tiene su propio periodo, si no, cuando esta en free running se cambia a algun lfo
  void setRatio(uint8_t lfoNum, float ratio);
  void resetPhase(uint8_t lfoNum);
  void resetPhaseMaster();
  void setWave(uint8_t lfoNum, byte wave);
  void enableSync(uint8_t lfoNum);  // si recibe o no sync externo
  void disableSync(uint8_t lfoNum);
  void setTriggerPeriod(uint8_t lfoNum, uint16_t triggerPeriod);
  void setTriggerPolarity(uint8_t lfoNum, bool triggerPolarity);
  void turnFreeRunning(uint8_t lfoNum, bool toggle);
  int getLfoValues(uint8_t lfoNum);
  int getLfoValuesPWM(uint8_t lfoNum);
  bool getClockOut();
  // void syncEnabled(uint8_t lfoNum);
 private:
  volatile uint8_t _lfoPin1;
  volatile uint8_t _lfoPin2;
  volatile uint8_t _syncPin1;
  volatile uint8_t _syncPin2;
  volatile uint8_t _clockOutPin = 19;
  uint32_t _ticksCycle;
  uint32_t _tableSizeFixedPoint;
  uint32_t _tableSize;
  volatile bool _syncEnabled[2] = {false, false};
  volatile uint32_t _output[2];
  volatile uint8_t _waveSelector[2];
  volatile uint32_t _phaseInc[2];
  volatile uint32_t _phaseAcc[2];
  volatile uint32_t _phaseAccClockOut;
  volatile uint32_t _phaseIncClockOut;
  volatile uint32_t _phaseAccMaster;
  volatile uint32_t _phaseIncMaster;
  volatile uint16_t _phaseAccClockOut12b;
  volatile uint16_t _phaseAcc12b[2];
  volatile uint32_t _phaseAccMaster12b;
  volatile uint32_t _masterTicks;
  ;
  float _compensation;
  volatile uint16_t _lfoPins[2];
  volatile uint16_t _syncPins[2];
  volatile bool _randomFlag[2];
  volatile float _ratio[2] = {1, 1};
  volatile float _ratio24[2] = {1, 1};
  volatile float _lastRatio[2] = {1, 1};
  volatile float _period[2] = {1, 1};
  volatile float _periodMaster = 1;
  volatile uint16_t _bitshift;
  volatile uint16_t _range;
  volatile uint16_t _rangeShift;
  volatile uint8_t _ledShift;
  volatile uint32_t _triggerPeriod[2] = {200, 200};  // triggerperiod y counter se encargan del duty cycle. En samples del samplerate
  volatile uint32_t _triggerCounter[2];
  volatile uint32_t _triggerCounterClockOut;
  volatile uint32_t _triggerPeriodClockOut = 200;
  ;
  volatile bool _clockOutValue;
  volatile bool _triggerOn[2] = {1, 1};
  volatile bool _triggerOff[2] = {0, 0};
  volatile bool _freeRunning[2] = {0, 0};
  volatile bool _flagTrigger[2] = {false, false};
  volatile bool _flagTriggerClockOut;
  void computeWaveforms();
  void syncOut();
};
// const int Lfo::bitshift = 17; //4096
/*
class Lfo {
   public:
    Lfo(volatile byte lfoPin, uint32_t sampleRate, uint32_t tableSize);
    void setFreqHz(float freq);
    void update();
    // int getValue();
    void setPeriodMs(float period);
    void resetPhase();
    void setWave(byte wave);
    int getPhase();

   private:
    byte _lfoPin;
    uint32_t _ticksCycle;
    uint32_t _tableSizeFixedPoint;
    uint32_t _tableSize;
    volatile uint32_t _output;
    volatile byte _waveSelector = 1;
    volatile uint32_t _phaseInc = 0;
    volatile uint32_t _phaseAcc = 0;
    const static int _bitShift = 17;
    volatile uint32_t _phaseAcc12b;

};
   //const int Lfo::bitshift = 17; //4096

*/
#endif