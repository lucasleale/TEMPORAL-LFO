#ifndef Lfo_h
#define Lfo_h
#include <Arduino.h>

#include "pico/stdlib.h"
#include "wavetables.h"




class Lfo {
     public:
     Lfo(volatile uint8_t lfoPin1, volatile uint8_t lfoPin2, uint32_t sampleRate, uint32_t tableSize);
     void setFreqHz(uint8_t lfoNum, float freq);
     void update();
     void setPeriodMs(uint8_t lfoNum, float period);
     void setRatio(uint8_t lfoNum, float ratio);
     void resetPhase(uint8_t lfoNum);
     void setWave(uint8_t lfoNum, byte wave);

     private :
    volatile uint8_t _lfoPin1;
    volatile uint8_t _lfoPin2;
    uint32_t _ticksCycle;
    uint32_t _tableSizeFixedPoint;
    uint32_t _tableSize;
   
    volatile uint32_t _output[2];
    volatile uint8_t _waveSelector[2];
    volatile uint32_t _phaseInc[2];
    volatile uint32_t _phaseAcc[2];
    volatile uint16_t _phaseAcc12b[2];
    volatile uint16_t _lfoPins[2];
    volatile bool _randomFlag[2];
    volatile float _ratio[2] = {1, 1};
    volatile float _lastRatio[2] = {1, 1};
    volatile float _period;
    const static int _bitShift = 17;
    void computeWaveforms();
    
};
//const int Lfo::bitshift = 17; //4096
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