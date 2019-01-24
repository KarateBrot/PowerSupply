#include "Stopwatch.h"


uint32_t Stopwatch::lifetime = 0;


Stopwatch::Stopwatch(std::function<uint32_t(void)> timeFn) :
  
  _fnTime(timeFn), 
  _time(_fnTime()) {
}