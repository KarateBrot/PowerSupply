#ifndef STOPWATCH_H
#define STOPWATCH_H

#include "Arduino.h"


class Stopwatch {

private:
  uint32_t _time, _tick;

public:
  static uint32_t lifetime;

  Stopwatch (void) { _time    = micros();         }
  ~Stopwatch(void) { lifetime = micros() - _time; }

  void     reset  (void)       { _time = micros();        }
  uint32_t getTime(void) const { return micros() - _time; }

  void     tick   (void)       { _tick++;      }
  uint32_t getTick(void) const { return _tick; }
};


#endif // STOPWATCH_H
