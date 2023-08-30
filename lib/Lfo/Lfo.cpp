#include "Lfo.h"
// #include "wavetables.h"
#include <Arduino.h>

#include "pico/stdlib.h"

Lfo::Lfo(volatile byte lfoPin1, volatile byte lfoPin2, uint32_t sampleRate,
         uint32_t tableSize, uint16_t range) {
  pinMode(lfoPin1, OUTPUT);
  pinMode(lfoPin2, OUTPUT);
  _lfoPin1 = lfoPin1;
  _lfoPin2 = lfoPin2;
  _lfoPins[0] = _lfoPin1;
  _lfoPins[1] = _lfoPin2;
  _tableSize = tableSize;
  _tableSizeFixedPoint = (_tableSize << 17);
  _ticksCycle = (float)((float)_tableSizeFixedPoint / float(sampleRate));
  
  _rangeOutput = 12 - (log((range+1)) / log(2));//esto hay que mejorar despues, el 12 son 12 bit. Tengo que ajustar 
  //los acumuladores y tabla de ondas para que sean de 16bit (65536) y ahi hacer esta comparacion para descartar
  //bits al final

  
}

void Lfo::update() {
  for (int lfoN = 0; lfoN < 2; lfoN++) {
    /////incrementar acumulador segun freq (phaseInc) y ratio
    _phaseAcc[lfoN] += _phaseInc[lfoN];  //* _ratio[lfoN];
    _phaseAcc12b[lfoN] = _phaseAcc[lfoN] >> 17; // >> 17 acorde a table size

    switch (_waveSelector[lfoN]) {  ////////////calculo formas de onda
      case 0:
        _randomFlag[lfoN] = false;
        _output[lfoN] = SIN[_phaseAcc12b[lfoN]];  // sin
        break;
      case 1:
        _randomFlag[lfoN] = false;
        _output[lfoN] = _phaseAcc12b[lfoN];  // saw
        break;
      case 2:
        _randomFlag[lfoN] = false;
        _output[lfoN] = (_tableSize - 1) - _phaseAcc12b[lfoN];  // inv saw
        break;
      case 3:
        _randomFlag[lfoN] = false;
        _output[lfoN] = TRI[_phaseAcc12b[lfoN]];  // triangle
        break;
      case 4:
        _randomFlag[lfoN] = false;
        _output[lfoN] = (_phaseAcc12b[lfoN] < (_tableSize / 2)) ? _tableSize - 1 : 0;  // square
        break;
      case 5:
        _randomFlag[lfoN] = true;  // el random se ejecuta en el hardsync/reset
        break;
      case 6:
        _randomFlag[lfoN] = false;
        _output[lfoN] = 4095;  // hold
        break;
    }
    ///////////////////

    ////////cuando pasa el tamanio de la tabla vuelve a 0, aca se calculan varias cosas por tema de hard sync
    if (_phaseAcc[lfoN] > _tableSizeFixedPoint) {
      if (abs(_phaseAcc12b[0] - _phaseAcc12b[1]) <= 128) { //este 128 hay que reemplazarlo por un valor que sea un ratio
      //entre ratio1 y ratio2. Mientras mas grande es la diferencia mas grande es el valor. x4 y 0.25 en 128 queda bien.
        // 1-lfoN invierte indice porque actua sobre el otro lfo

        _phaseAcc[1 - lfoN] += _tableSizeFixedPoint;  // hard sync
        // en vez de resetear a 0 le sumamos para que supere _tablesizefix.
        // esto soluciona un error con el random
      }

      if (_lastRatio[1 - lfoN] != _ratio[1 - lfoN]) {  // if (phaseIncChange) {
        _phaseAcc[1 - lfoN] = 0;
        _lastRatio[1 - lfoN] = _ratio[1 - lfoN];
      }
      if (_randomFlag[lfoN]) {  // random refresca valor en el momento del hardsync. Si no, nunca sucede por el fix de la wavetable.
        _output[lfoN] = random(0, 4096);
        analogWrite(_lfoPins[lfoN], _output[lfoN] >> _rangeOutput);
      }
      _phaseAcc[lfoN] = 0;
    } else {  // refrescar el lfo siempre menos en el momento del hardsync ya que glitchea en la tabla de ondas
      analogWrite(_lfoPins[lfoN], _output[lfoN] >> _rangeOutput);
    }
  }
  // analogWrite(_lfoPin1, _output[0]);
  // analogWrite(_lfoPin2, _output[1]);
}

void Lfo::setFreqHz(uint8_t lfoNum, float freq) {
  _phaseInc[lfoNum] = _ticksCycle * freq;
}

void Lfo::setPeriodMs(uint8_t lfoNum, float period) {
  // float freqFromPeriod = 1000. / period;
  // setFreqHz(freqFromPeriod);
  _period = period;
  _phaseInc[lfoNum] = _ticksCycle * (1000. / (period * _ratio[lfoNum]));
}

void Lfo::setRatio(uint8_t lfoNum, float ratio) {
  _ratio[lfoNum] = ratio;
  _phaseInc[lfoNum] = _ticksCycle * (1000. / (_period * ratio));
}

void Lfo::setWave(uint8_t lfoNum, uint8_t wave) {
  _waveSelector[lfoNum] = wave;
}

void Lfo::resetPhase(uint8_t lfoNum) {
  noInterrupts();
  _phaseAcc[lfoNum] = 0;
  interrupts();
}
/*
Lfo::Lfo(volatile byte lfoPin, uint32_t sampleRate, uint32_t tableSize) {
    pinMode(lfoPin, OUTPUT);
    _lfoPin = lfoPin;
    _tableSize = tableSize;
    _tableSizeFixedPoint = (_tableSize << _bitShift);
    _ticksCycle = (float)((float)_tableSizeFixedPoint / float(sampleRate));
}

void Lfo::setFreqHz(float freq) {
    _phaseInc = _ticksCycle * freq;
    //
}

void Lfo::update() {
    _phaseAcc += _phaseInc;
    if (_phaseAcc > _tableSizeFixedPoint) {
        _phaseAcc -= _tableSizeFixedPoint;
    }
    _phaseAcc12b = _phaseAcc >> _bitShift;
    switch (_waveSelector) {
        case 0:
            _output = SAWUP[_phaseAcc12b];
            break;
        case 1:
            _output = 4095 - SAWUP[_phaseAcc12b];  // sawdown
            break;
        case 2:
            _output = TRI[_phaseAcc12b];
            break;
        case 3:
            _output = (_phaseAcc12b < (_tableSize / 2)) ? _tableSize - 1 : 0;

            break;
        case 4:
            if (_phaseAcc12b == 0) {
                _output = random(0, 4096);
            }
    }

    analogWrite(_lfoPin, _output);
}

void Lfo::setPeriodMs(float period) {
    //float freqFromPeriod = 1000. / period;
    //setFreqHz(freqFromPeriod);
    _phaseAcc = _ticksCycle * (1000. / period);
}

void Lfo::resetPhase() {
    noInterrupts();
    _phaseAcc = 0;
    interrupts();
}

int Lfo::getPhase() { return _phaseAcc12b; }

void Lfo::setWave(byte wave) { _waveSelector = wave; }
*/