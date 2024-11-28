#include "Lfo.h"
// #include "wavetables.h"
#include <Arduino.h>

#include "pico/stdlib.h"
#define DOUBLE_TRIGGER_THRESH 6  // 6 = 1/16th
Lfo::Lfo(volatile byte lfoPin1, volatile byte lfoPin2, uint32_t sampleRate,
         uint32_t tableSize, uint16_t range, uint8_t syncPin1, uint8_t syncPin2) {
  pinMode(lfoPin1, OUTPUT);
  pinMode(lfoPin2, OUTPUT);
  pinMode(syncPin1, OUTPUT);
  pinMode(syncPin2, OUTPUT);
  pinMode(_clockOutPin, OUTPUT);
  _compensation = 1.0013;  // 1.0013;  // 1.0023  queda bien en con el lfo en el loop, 500.034ms avg We. Ahora queda bien con el lfo aca adentro.
  // we, cuando sume la parte de clockout (solo el calculo de modulo con el digitalwrt), en vez de 500 me da 499.6. Dont know why. Ajustar
  // con sync, clock y lfo adentro 1.0014 queda clavado en 500.005ms
  // a veces se mueve no se xq. Ahora 1.0013 queda mejor zzzz.
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

  _rangeShift = 12 - (log((range + 1)) / log(2));  // esto hay que mejorar despues, el 12 son 12 bit. Tengo que ajustar
                                                   // los acumuladores y tabla de ondas para que sean de 16bit (65536) y ahi hacer esta comparacion para descartar
                                                   // bits al final
  _ledShift = 4 - _rangeShift;                     // si range es 4095, rangeshift es 0 y shiftea 0, para los leds tiene que shift 4 (4-4)
                                                   // si range es 1023, rangeshift es 2 y shiftea 2 para los leds, 4-2 = 2.
}

void Lfo::update() {
  _phaseAccMaster += _phaseIncMaster;

  if (!_syncEnabled[0] && !_syncEnabled[1]) {  // todo este bloque se ejecuta cuando ninguno esta en sync, cuando esta como master.
    // para sync usa el algoritmo viejo. El clock out en sync tambien es aparte.
    if (!_freeRunning[0]) {
      _phaseAcc[0] += _phaseInc[0];
      _phaseAcc12b[0] = _phaseAcc[0] >> 17;
    }
    if (!_freeRunning[1]) {
      _phaseAcc[1] += _phaseInc[1];
      _phaseAcc12b[1] = _phaseAcc[1] >> 17;
    }

    if (_phaseAccMaster > _tableSizeFixedPoint) {  // master phasor. Va x24 veces mas rapido. Los lfo1 y lfo2 derivan de aca contando cada cycleÏ
      // Serial.print(_masterTicks % 24);
      // Serial.print("   ");
      // Serial.println(_phaseAcc[0]);
      _phaseAccMaster -= _tableSizeFixedPoint;
      _masterTicks++;
      for (int lfoN = 0; lfoN < 2; lfoN++) {
        if (!_freeRunning[lfoN]) {
          if (_masterTicks % int(ceil(_ratio24[lfoN])) == 0) {  // masterticks incrementa 1 en cada ciclo del master, con modulo derivan slaves
            _phaseAcc[lfoN] = 0;

            _phaseAcc12b[lfoN] = 0;
            if (_randomFlag[lfoN]) {
              _output[lfoN] = random(0, 4096);
              analogWrite(_lfoPins[lfoN], _output[lfoN] >> _rangeShift);
            }
          }
        }
      }
      if (_extClock == false) {
        if (_masterTicks % (24 / _ppqn) == 0) {
          _triggerCounterClockOut = 0;
          _ppqnCount;  // para separar led de clock out ppqn sin hacer nada nuevo

          digitalWrite(_clockOutPin, _clockOn);  // new
          _ppqnCount++;
          if (_ppqnCount % _ppqn == 0) {
            _clockOutValueLed = 1;  // always POS
          }
          

          _clockOutValue = _clockOn;

          _flagTriggerClockOut = true;
        }
      }
    }
    if (_extClock == false) {
      if ((_triggerCounterClockOut > _triggerPeriodClockOut) && _flagTriggerClockOut) {
        digitalWrite(_clockOutPin, _clockOff);
        _clockOutValue = _clockOff;
        _clockOutValueLed = 0;  // always NEG
        _triggerCounterClockOut = 0;
        _flagTriggerClockOut = false;
      }
      _triggerCounterClockOut++;
    }

    for (int lfoN = 0; lfoN < 2; lfoN++) {
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
      if (_phaseAcc[lfoN] > _tableSizeFixedPoint) {
        _phaseAcc[lfoN] = 0;
        _phaseAcc12b[lfoN] = 0;
      }

      if (!_randomFlag[lfoN]) {
        analogWrite(_lfoPins[lfoN], _output[lfoN] >> _rangeShift);
      }
    }

    // SYNC OUT
    for (int lfoN = 0; lfoN < 2; lfoN++) {
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
    }
  } else {
    if (_phaseAccMaster > _tableSizeFixedPoint) {  // master phasor. Va x24 veces mas rapido. Los lfo1 y lfo2 derivan de aca contando cada cycleÏ
      _phaseAccMaster -= _tableSizeFixedPoint;
      _masterTicks++;
      if (_masterTicks % 24 == 0) {
        _triggerCounterClockOut = 0;
        if (_extClock == false) {
          // digitalWrite(_clockOutPin, HIGH);

          //_clockOutValue = 1;
        }
        _flagTriggerClockOut = true;
      }
    }
    if ((_triggerCounterClockOut > _triggerPeriodClockOut) && _flagTriggerClockOut) {
      if (_extClock == false) {
        // digitalWrite(_clockOutPin, LOW);

        //_clockOutValue = LOW;
      }
      _triggerCounterClockOut = 0;
      _flagTriggerClockOut = false;
    }
    _triggerCounterClockOut++;
    // Serial.println("codigo clock from ext aca");
    /*if ((_triggerCounterClockOut > _triggerPeriodClockOut) && _flagTriggerClockOut) {
      digitalWrite(_clockOutPin, LOW);
      _clockOutValue = LOW;
      _triggerCounterClockOut = 0;
      _flagTriggerClockOut = false;
    }
    _triggerCounterClockOut++;*/
  }

  /*if (_phaseAccClockOut == 0) {
    _triggerCounterClockOut = 0;
    digitalWrite(_clockOutPin, HIGH);
    _clockOutValue = HIGH;
    _flagTriggerClockOut = true;
  }
  _phaseAccClockOut += _phaseIncClockOut;

  if ((_triggerCounterClockOut > _triggerPeriodClockOut) && _flagTriggerClockOut) {
    digitalWrite(_clockOutPin, LOW);
    _clockOutValue = LOW;
    _triggerCounterClockOut = 0;
    _flagTriggerClockOut = false;
  }
  _triggerCounterClockOut++;
  if (_phaseAccClockOut > _tableSizeFixedPoint) {  // al final para hardsync y eso usamos como base el master clock out.
    //////////CLOCKOUT
    if (_freeRunning[0] == false) {
      if (_lastRatio[0] != _ratio[0]) {  // if (phaseIncChange) { //mmm para el cambio de ratio y lock no funciona bien aca
        //_phaseAcc[0] = 0;                // cycle mode force reset

        //_lastRatio[0] = _ratio[0];
      }
    }
    if (_freeRunning[1] == false) {
      if (_lastRatio[1] != _ratio[1]) {  // if (phaseIncChange) {
        //_phaseAcc[1] = 0;                // cycle mode force reset

        //_lastRatio[1] = _ratio[1];
      }
    }
    ///////////////////
    _phaseAccClockOut = 0;
  }*/
  // analogWrite(_lfoPins[0], _output[0] >> _rangeShift);
  // analogWrite(_lfoPins[0], SIN[_phaseAcc12b[0]] >> _rangeShift);

  for (int lfoN = 0; lfoN < 2; lfoN++) {
    if ((_syncEnabled[lfoN] || _freeRunning[lfoN])) {
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
        // la comparacion del abs es por el error de calculo de la DDS. Si estan a ratios muy distantes se van a desfasar,
        // con la comparacion chequeamos esa diferencia y si esta entre el margen de error, fuerza reset.
        if (abs(_phaseAcc12b[0] - _phaseAcc12b[1]) <= 1) {  // este 128 hay que reemplazarlo por un valor que sea un ratio
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
        ////queda mejor aca que en la parte de master clock, para que no se desfase cuando cambia de ratio
        if (_freeRunning[1 - lfoN] == false || _freeRunning[lfoN] == false) {
          if (_lastRatio[1 - lfoN] != _ratio[1 - lfoN]) {  // if (phaseIncChange) {
            _phaseAcc[1 - lfoN] = 0;                       // cycle mode force reset

            _lastRatio[1 - lfoN] = _ratio[1 - lfoN];
          }
        }

        if (_randomFlag[lfoN] && !_syncEnabled[lfoN]) {  // random refresca valor en el momento del hardsync. Si esta en sync externo no genera aca.
          _output[lfoN] = random(0, 4096);
          analogWrite(_lfoPins[lfoN], _output[lfoN] >> _rangeShift);
        }
        // Serial.println(_phaseAcc[lfoN]);
        _phaseAcc[lfoN] = 0;

      } else {
        if (!_randomFlag[lfoN]) {  // refrescar el lfo siempre menos en el momento del hardsync ya que glitchea en la tabla de ondas
          analogWrite(_lfoPins[lfoN], _output[lfoN] >> _rangeShift);
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
  }

  // analogWrite(_lfoPin1, _output[0]);
  // analogWrite(_lfoPin2, _output[1]);
}

int Lfo::getLfoValues(uint8_t lfoNum) {
  if (lfoNum == 0) {
    return (_output[0] >> _rangeShift) >> _ledShift;
  } else if (lfoNum == 1) {
    return (_output[1] >> _rangeShift) >> _ledShift;
  } else {
    return -1;
  }
}
int Lfo::getLfoValuesPWM(uint8_t lfoNum) {
  if (lfoNum == 0) {
    return (_output[0] >> _rangeShift);
  } else if (lfoNum == 1) {
    return (_output[1] >> _rangeShift);
  } else {
    return -1;
  }
}

bool Lfo::getClockOut() {
  if (_extClock || _midiClock) {
    return _clockOutValue;
  } else {
    return _clockOutValueLed;
  }
}
void Lfo::setFreqHz(uint8_t lfoNum, float freq) {
  _phaseInc[lfoNum] = _ticksCycle * freq;
}

void Lfo::setPeriodMs(uint8_t lfoNum, float period) {
  // float freqFromPeriod = 1000. / period;
  // setFreqHz(freqFromPeriod);
  //_periodMaster = period;
  _period[lfoNum] = period;
  //_phaseIncMaster = (_ticksCycle * (1000. / period) * 24) * _compensation;                        // 24x rapido
  _phaseInc[lfoNum] = _ticksCycle * (1000. / period) * (24. / _ratio24[lfoNum]) * _compensation;  // 24 / 12 por ej. Ratio interno es 12, 24, etc. En main 0.5
  //_phaseInc[lfoNum] = (_ticksCycle * (1000. / (period * _ratio[lfoNum])));  //*1.004
}

void Lfo::setPeriodMsClock(float period) {
  _phaseIncMaster = (_ticksCycle * (1000. / period) * 24) * _compensation;  // para que el Msclock siga en la suya si el resto esta en free running.
  // Serial.println(_phaseIncMaster);
  //  Serial.println(_phaseIncMaster);
}

void Lfo::setExtClock(bool state) {
  if (state) {
    _extClock = true;
  } else {
    _extClock = false;
  }
  Serial.println(_extClock);
}

void Lfo::setRatio(uint8_t lfoNum, float ratio) {
  //_ratio[lfoNum] = ratio;
  _ratio24[lfoNum] = ratio * 24.;  // nuevo codigo x24 asi el ratio queda en 6, 12, 24, etc, para el modulo
  //_phaseAcc[lfoNum] = _phaseAcc[1 - lfoNum] / ratio;                        // modo lock para que los sync no tiren retrigger. Divido porque accum es inversa de F
  //_phaseInc[lfoNum] = (_ticksCycle * (1000. / (_period[lfoNum] * ratio)));  //*1.004
  _phaseInc[lfoNum] = _ticksCycle * (1000. / _period[lfoNum]) * (24. / _ratio24[lfoNum]) * _compensation;
}

void Lfo::setWave(uint8_t lfoNum, uint8_t wave) {
  _waveSelector[lfoNum] = wave;
}

void Lfo::resetPhaseMaster() {
  //_phaseAccClockOut = 0;  // esto lo sacamos de resetPhase para que resetee separado del resetphase
  // si no se triggeaba con cada reset de la subdivision

  /*_phaseAcc[0] = 0;
  _phaseAcc[1] = 0;
  _flagTrigger[1] = true;
  _triggerCounter[1] = 0;
  _flagTrigger[0] = true;
  _triggerCounter[0] = 0;
  digitalWrite(_syncPins[0], _triggerOn[0]);
  digitalWrite(_syncPins[1], _triggerOn[1]);*/
  /*static int _counterReset;
  if(_counterReset % 4 == 0){
  //_phaseAcc[0] = 0;
  //analogWrite(_lfoPins[0], _output[0] >> _rangeShift);
  _phaseAccMaster = 0;  // con esto activo solo, glitchea la forma de onda. Si activo phaseAcc = 0, no.
  // si desactivo todo lo de arriba  dejo solo masterticks = 0 funciona 100%. Pero quiero probar de resetear todo full y ver que onda.
  // porque ahora vi y el problema es cunado la subdivision es mayor a 1/4, como resetea siempre, la corta. mmmm
  _masterTicks = 0;
  _triggerCounterClockOut = 0;  // esto es importante para que no se olvide ningun pulso *1

  digitalWrite(_clockOutPin, HIGH);
  _clockOutValue = 1;
  _flagTriggerClockOut = true;
  }

  _counterReset++;*/

  _masterTicks = 0;  // comentado 22/10/24
  _phaseAccMaster = 0;
  _ppqnCount = 0;

  /*_triggerCounterClockOut = 0;  // esto es importante para que no se olvide ningun pulso *1
  digitalWrite(_clockOutPin, HIGH);
  _clockOutValue = 1;
  _flagTriggerClockOut = true;*/

  // analogWrite(_lfoPins[0], _output[0] >> _rangeShift);
  // analogWrite(_lfoPins[1], _output[1] >> _rangeShift);
  //  aca ya ni se, cada tanto fallaba algun pulso.

  //_triggerCounterClockOut = 0;
  // ya me hice un quilombo aca. El problema es un glitch en la forma de onda, pero para mi que es inaudible.
  // asi que dejo esto asi
}
void Lfo::triggerReset() {
  // Serial.println(_masterTicks % 24);
  if (_masterTicks % 24 > DOUBLE_TRIGGER_THRESH) {  // evita doble trigger
    _masterTicks = 0;
    _phaseAccMaster = 0;
    _triggerCounterClockOut = 0;  // esto es importante para que no se olvide ningun pulso *1
    digitalWrite(_clockOutPin, _clockOn);

    _clockOutValue = 1;
    _ppqnCount = 0;
    _clockOutValueLed = 1;
    _flagTriggerClockOut = true;
  }
}
void Lfo::clockFromExt() {
  // Serial.println(_masterTicks % 24);

  _masterTicks = 0;
  _phaseAccMaster = 0;
  _triggerCounterClockOut = 0;  // esto es importante para que no se olvide ningun pulso *1
  _ppqnCount = 0;
  // digitalWrite(_clockOutPin, HIGH);
  _clockOutValue = 1;
  _flagTriggerClockOut = true;
}
void Lfo::resetPhase(uint8_t lfoNum) {
  // noInterrupts();

  // if (_randomFlag[lfoNum]) {
  //_phaseAcc[lfoNum] += _tableSizeFixedPoint;  // si reseteamos en 0 no sucede el random.
  //} else {
  //_phaseAcc[lfoNum] = 0;
  _flagTrigger[lfoNum] = true;  // con esto no le pifia a ningun syncout *1
  _triggerCounter[lfoNum] = 0;
  digitalWrite(_syncPins[lfoNum], _triggerOn[lfoNum]);

  _masterTicks = 0;
  _phaseAcc[lfoNum] = 0;
  _phaseAcc12b[lfoNum] = 0;
  _ppqnCount = 0;
  _phaseAccMaster = 0;
  //_output[lfoNum] = random(0, 4096);
  // analogWrite(_lfoPins[lfoNum], _output[lfoNum] >> _rangeShift);
  //_masterTicks = 0;
  //_triggerCounter[0] = 0;
  //_triggerCounter[1] = 0;
  //_phaseAccClockOut = 0;
  // }
  //  interrupts();
  if (_syncEnabled[lfoNum]) {  // si recibe sync externo genera el random nuevo directamente en el reset. Ya que si no
    // a veces genera un pico de glitch random porque genera un nuevo valor cuando resetea con clock, y otro si el acumulador resetea solo.
    if (_randomFlag[lfoNum]) {
      _output[lfoNum] = random(0, 4096);
      analogWrite(_lfoPins[lfoNum], _output[lfoNum] >> _rangeShift);
    }
  }
}
void Lfo::enableSync(uint8_t lfoNum) {
  _syncEnabled[lfoNum] = true;
}
void Lfo::disableSync(uint8_t lfoNum) {
  _syncEnabled[lfoNum] = false;
}
void Lfo::enableMidi(uint8_t lfoNum) {
  _midiEnabled[lfoNum] = true;
  _midiClock = true;
}
void Lfo::disableMidi(uint8_t lfoNum) {
  _midiEnabled[lfoNum] = false;
  _midiClock = false;
}
void Lfo::turnFreeRunning(uint8_t lfoNum, bool toggle) {
  if (lfoNum < 2) {
    _freeRunning[lfoNum] = toggle;
  }
  if (_freeRunning[lfoNum] == true) {
    //_ratio[lfoNum] = 1 * 24.; //x24 lfo
  } /*else {
    _phaseAcc[lfoNum] = _phaseAcc[1 - lfoNum] / _ratio[lfoNum];  // aca forzamos a que el lfo salte a donde deberia estar cuando vuelve
    // a ratios para uqe no quede fuera de fase
  }*/
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
void Lfo::setClockPolarity(bool clockPolarity) {
  _clockOn = clockPolarity;
  _clockOff = 1 - clockPolarity;
}
void Lfo::setClockPPQN(byte ppqn) {
  _ppqn = ppqn;
}

void Lfo::clockOut(bool state) {
  if (_extClock || _midiClock) {
    digitalWrite(_clockOutPin, state);
    _clockOutValue = state;
  }
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