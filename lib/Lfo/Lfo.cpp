#include "Lfo.h"
// #include "wavetables.h"
#include <Arduino.h>

#include "pico/stdlib.h"

Lfo::Lfo(volatile byte lfoPin1, volatile byte lfoPin2, uint32_t sampleRate,
         uint32_t tableSize, uint16_t range, uint8_t syncPin1, uint8_t syncPin2) {
  pinMode(lfoPin1, OUTPUT);
  pinMode(lfoPin2, OUTPUT);
  pinMode(syncPin1, OUTPUT);
  pinMode(syncPin2, OUTPUT);
  pinMode(_clockOutPin, OUTPUT);
  _lfoPin1 = lfoPin1;
  _lfoPin2 = lfoPin2;
  _syncPin1 = syncPin1;
  _syncPin2 = syncPin2;
  _lfoPins[0] = _lfoPin1;
  _lfoPins[1] = _lfoPin2;
  _syncPins[0] = _syncPin1;
  _syncPins[1] = _syncPin2;
  _tableSize = tableSize;
  _tableSizeFixedPoint = (_tableSize << 17);
  _ticksCycle = (float)((float)_tableSizeFixedPoint / float(sampleRate));

  _rangeOutput = 12 - (log((range + 1)) / log(2));  // esto hay que mejorar despues, el 12 son 12 bit. Tengo que ajustar
  // los acumuladores y tabla de ondas para que sean de 16bit (65536) y ahi hacer esta comparacion para descartar
  // bits al final
}

void Lfo::update() {

  if (_phaseAccClockOut == 0) {
    _triggerCounterClockOut = 0;
    digitalWrite(_clockOutPin, HIGH);
    _flagTriggerClockOut = true;
  }
  _phaseAccClockOut += _phaseIncClockOut;

  if ((_triggerCounterClockOut > _triggerPeriodClockOut) && _flagTriggerClockOut) {
    digitalWrite(_clockOutPin, LOW);
    _triggerCounterClockOut = 0;
    _flagTriggerClockOut = false;
  }
  _triggerCounterClockOut++;
  if (_phaseAccClockOut > _tableSizeFixedPoint) {  // al final para hardsync y eso usamos como base el master clock out.
    //////////CLOCKOUT
    if (_freeRunning[0] == false) {
      if (_lastRatio[0] != _ratio[0]) {  // if (phaseIncChange) {
        _phaseAcc[0] = 0;                // cycle mode force reset

        _lastRatio[0] = _ratio[0];
      }
    }
    if (_freeRunning[1] == false) {
      if (_lastRatio[1] != _ratio[1]) {  // if (phaseIncChange) {
        _phaseAcc[1] = 0;                // cycle mode force reset

        _lastRatio[1] = _ratio[1];
      }
    }
    ///////////////////
    _phaseAccClockOut = 0;
  }

  for (int lfoN = 0; lfoN < 2; lfoN++) {
    /////SYNC OUT
    if (_phaseAcc[lfoN] == 0) {
      _triggerCounter[lfoN] = 0;
      digitalWrite(_syncPins[lfoN], _triggerOn[lfoN]);

      _flagTrigger[lfoN] = true;
      // Serial.print("trig on");
      // Serial.println(random(0,100));
    }

    if ((_triggerCounter[lfoN] > _triggerPeriod[lfoN]) && _flagTrigger[lfoN]) {
      digitalWrite(_syncPins[lfoN], _triggerOff[lfoN]);
      _triggerCounter[lfoN] = 0;
      _flagTrigger[lfoN] = false;
      // Serial.print("trig off");
      // Serial.println(random(0,100));
    }
    _triggerCounter[lfoN]++;
    /////////////

    /////incrementar acumulador segun freq (phaseInc) y ratio
    _phaseAcc[lfoN] += _phaseInc[lfoN];          //* _ratio[lfoN];
    _phaseAcc12b[lfoN] = _phaseAcc[lfoN] >> 17;  // >> 17 acorde a table size
    //_phaseAccClockOut += _phaseIncClockOut;
    //_phaseAccClockOut12b = _phaseAccClockOut >> 17;

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
      if (abs(_phaseAcc12b[0] - _phaseAcc12b[1]) <= 128) {  // este 128 hay que reemplazarlo por un valor que sea un ratio
        // entre ratio1 y ratio2. Mientras mas grande es la diferencia mas grande es el valor. x4 y 0.25 en 128 queda bien.
        //  1-lfoN invierte indice porque actua sobre el otro lfo
        if (_freeRunning[1 - lfoN] == false || _freeRunning[lfoN] == false) {  // hardsync solo pasa cuando estan en subdivisiones.
          _phaseAcc[1 - lfoN] += _tableSizeFixedPoint;                         // hard sync
        }
        // en vez de resetear a 0 le sumamos para que supere _tablesizefix.
        // esto soluciona un error con el random
      }
      //////////CLOCKOUT
      /*if (abs(_phaseAcc12b[0] - _phaseAccClockOut12b) <= 128) {
        if (_freeRunning[1 - lfoN] == false || _freeRunning[lfoN] == false) {  // hardsync solo pasa cuando estan en subdivisiones.
          _phaseAccClockOut += _tableSizeFixedPoint;                           // hard sync
        }
      }
      //////////////////*/
      ////probando de sacar esto cuando caubmio ratio. Que el master sea el nuevo clock out?
      /*if (_freeRunning[1 - lfoN] == false || _freeRunning[lfoN] == false) {
        if (_lastRatio[1 - lfoN] != _ratio[1 - lfoN]) {  // if (phaseIncChange) {
          _phaseAcc[1 - lfoN] = 0;                       // cycle mode force reset

          _lastRatio[1 - lfoN] = _ratio[1 - lfoN];
        }
      }*/

      if (_randomFlag[lfoN] && !_syncEnabled[lfoN]) {  // random refresca valor en el momento del hardsync. Si esta en sync externo no genera aca.
        _output[lfoN] = random(0, 4096);
        analogWrite(_lfoPins[lfoN], _output[lfoN] >> _rangeOutput);
      }
      // Serial.println(_phaseAcc[lfoN]);
      _phaseAcc[lfoN] = 0;

    } else {
      if (!_randomFlag[lfoN]) {  // refrescar el lfo siempre menos en el momento del hardsync ya que glitchea en la tabla de ondas
        analogWrite(_lfoPins[lfoN], _output[lfoN] >> _rangeOutput);
      }
    }
    /*if(_phaseAccClockOut > _tableSizeFixedPoint){
      //////////CLOCKOUT
      if (abs(_phaseAcc12b[0] - _phaseAccClockOut12b) <= 128) {
        if (_freeRunning[1 - lfoN] == false || _freeRunning[lfoN] == false) {  // hardsync solo pasa cuando estan en subdivisiones.
          _phaseAccClockOut += _tableSizeFixedPoint;                           // hard sync
        }
      }
      ///////////////////
      _phaseAccClockOut = 0;
    }*/
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
  _period[lfoNum] = period;
  _phaseInc[lfoNum] = (_ticksCycle * (1000. / (period * _ratio[lfoNum])) * 1.004);
  
}

void Lfo::setPeriodMsClock(float period){
  _phaseIncClockOut = _ticksCycle * (1000. / period);  // clock sin ratio
}

void Lfo::setRatio(uint8_t lfoNum, float ratio) {
  _ratio[lfoNum] = ratio;
  _phaseAcc[lfoNum] = _phaseAcc[1 - lfoNum] / ratio;  // modo lock para que los sync no tiren retrigger. Divido porque accum es inversa de F
  _phaseInc[lfoNum] = (_ticksCycle * (1000. / (_period[lfoNum] * ratio)) * 1.004);
}

void Lfo::setWave(uint8_t lfoNum, uint8_t wave) {
  _waveSelector[lfoNum] = wave;
}

void Lfo::resetPhase(uint8_t lfoNum) {
  // noInterrupts();

  // if (_randomFlag[lfoNum]) {
  //_phaseAcc[lfoNum] += _tableSizeFixedPoint;  // si reseteamos en 0 no sucede el random.
  //} else {
  _phaseAcc[lfoNum] = 0;
  _phaseAccClockOut = 0;
  //}
  // interrupts();
  if (_syncEnabled[lfoNum]) {  // si recibe sync externo genera el random nuevo directamente en el reset. Ya que si no
    // a veces genera un pico de glitch random porque genera un nuevo valor cuando resetea con clock, y otro si el acumulador resetea solo.
    if (_randomFlag[lfoNum]) {
      _output[lfoNum] = random(0, 4096);
      analogWrite(_lfoPins[lfoNum], _output[lfoNum] >> _rangeOutput);
    }
  }
}
void Lfo::enableSync(uint8_t lfoNum) {
  _syncEnabled[lfoNum] = true;
}
void Lfo::disableSync(uint8_t lfoNum) {
  _syncEnabled[lfoNum] = false;
}
void Lfo::turnFreeRunning(uint8_t lfoNum, bool toggle) {
  if (lfoNum < 2) {
    _freeRunning[lfoNum] = toggle;
  }
  if (_freeRunning[lfoNum] == true) {
    _ratio[lfoNum] = 1;
  } else {
    _phaseAcc[lfoNum] = _phaseAcc[1 - lfoNum] / _ratio[lfoNum];  // aca forzamos a que el lfo salte a donde deberia estar cuando vuelve
    // a ratios para uqe no quede fuera de fase
  }
}

void Lfo::syncOut() {
}
void Lfo::setTriggerPeriod(uint8_t lfoNum, uint16_t triggerPeriod) {
  _triggerPeriod[lfoNum] = triggerPeriod;
}

void Lfo::setTriggerPolarity(uint8_t lfoNum, bool triggerPolarity) {
  _triggerOn[lfoNum] = triggerPolarity;
  _triggerOff[lfoNum] = 1 - triggerPolarity;
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