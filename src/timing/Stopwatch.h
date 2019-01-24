#ifndef STOPWATCH_H
#define STOPWATCH_H

#include <cstdint>
#include <functional>



class Stopwatch {

private:
  std::function<uint32_t(void)> _fnTime;
  uint32_t _time, _tick;   

public:
  static uint32_t lifetime;

  Stopwatch (std::function<uint32_t(void)>);
  ~Stopwatch(void) { lifetime = _fnTime() - _time; }

  void     reset  (void)       { _time = _fnTime();        }
  uint32_t getTime(void) const { return _fnTime() - _time; }

  void     tick   (void)       { _tick++;      }
  uint32_t getTick(void) const { return _tick; }
};



#endif // STOPWATCH_H
