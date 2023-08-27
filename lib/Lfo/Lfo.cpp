#include "Lfo.h"
// #include "wavetables.h"
#include <Arduino.h>

#include "pico/stdlib.h"

Lfo::Lfo(volatile byte lfoPin1, volatile byte lfoPin2, uint32_t sampleRate,
         uint32_t tableSize) {
  pinMode(lfoPin1, OUTPUT);
  pinMode(lfoPin2, OUTPUT);
  _lfoPin1 = lfoPin1;
  _lfoPin2 = lfoPin2;
  _tableSize = tableSize;
  _tableSizeFixedPoint = (_tableSize << _bitShift);
  _ticksCycle = (float)((float)_tableSizeFixedPoint / float(sampleRate));
}

void Lfo::update() {
  for (int lfoN = 0; lfoN < 2; lfoN++) {
    /////incrementar acumulador segun freq (phaseInc) y ratio
    _phaseAcc[lfoN] += _phaseInc[lfoN]; //* _ratio[lfoN];
    _phaseAcc12b[lfoN] = _phaseAcc[lfoN] >> 17;

    ////////cuando pasa el tamanio de la tabla vuelve a 0, aca se calculan varias cosas por tema de hard sync
    if (_phaseAcc[lfoN] > _tableSizeFixedPoint) {
      if (abs(_phaseAcc12b[0] - _phaseAcc12b[1]) <= 16) {  // 1-lfoN invierte indice porque actua
                                                           // hard sync
        _phaseAcc[1 - lfoN] += _tableSizeFixedPoint;       // en vez de resetear a 0 le sumamos para que supere _tablesizefix.
                                                           // esto soluciona un error con el random
      }

      if (_lastRatio[1 - lfoN] != _ratio[1 - lfoN]) {  // if (phaseIncChange) {
        _phaseAcc[1 - lfoN] = 0;
        _lastRatio[1 - lfoN] = _ratio[1 - lfoN];
      }

      _phaseAcc[lfoN] = 0;
    }
    //////calcular formas de onda aca
    switch (_waveSelector[lfoN]) {
      case 0:
        _output[lfoN] = _phaseAcc12b[lfoN];  // aca iria SINE, ahora es saw
        break;
      case 1:
        _output[lfoN] = (_tableSize - 1) - _phaseAcc12b[lfoN];  // inv saw
        break;
      case 2:
        _output[lfoN] = TRI[_phaseAcc12b[lfoN]];
        break;
      case 3:
        _output[lfoN] = (_phaseAcc12b[lfoN] < (_tableSize / 2)) ? _tableSize - 1 : 0;
        break;
      case 4:
        if (_phaseAcc[lfoN] == 0) {  // para random usar phaseAcc, no phaseAcc12b.
          _output[lfoN] = random(0, 4096);
        }
        break;
    }
    ///////////////////
  }
  analogWrite(_lfoPin1, _output[0]);
  analogWrite(_lfoPin2, _output[1]);
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