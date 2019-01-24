#include "Tools.h"

// Simple exponential smoothing
void Tools::smoothExp(double &x, const double &val, const float &weight) {

  x = (1.0-weight)*x + weight*val;
}


// Advanced exponential smoothing (simulates capacitor)
void Tools::smoothExp(double &x, const double &val, const uint32_t &sampleTime, const uint32_t &timeConst) {
  
  double weight = 1 - exp(-sampleTime/(double)timeConst);
  x = (1-weight)*x + weight*val;
}


// Hysteresis (simulates Inverting Schmitt Trigger)
bool Tools::trigger(bool &trigger, const double &val, const float &low, const float &high) {
  
  bool trigger_last = trigger;

  if (val > high) { trigger = true;  } else
  if (val < low ) { trigger = false; }

  if (trigger_last != trigger) { return true; } else { return false; }
}


// Constrains (and modifies) a value according to lower and upper limit
template <typename T, typename U>
T Tools::limit(const T &val, const U &low, const U &high) {

  if (val > high) { return (T)high; } else
  if (val < low ) { return (T)low;  } else
  return val;
}


// Constrains (and modifies) a value according to lower and upper limit
template <typename T, typename U>
bool Tools::trim(T &val, const U &low, const U &high) {

  if (val > high) { val = high; return true; } else
  if (val < low ) { val = low;  return true; } else
  return false;
}