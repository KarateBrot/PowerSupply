#include "tools.h"


// Check if value is a power of 2
bool tools::isPower2(const uint32_t &val) {

  return val && !(val & (val-1));
}

// log2 for integers with complexity O(N)
uint8_t tools::log2(uint32_t val) {

  uint8_t n = 0;
  while (val >>= 1) n++;
  return n;
}


// Simple exponential smoothing
void tools::smoothExp(double &x, const double &val, const float &weight) {

  x = (1.0-weight)*x + weight*val;
}


// Advanced exponential smoothing (simulates capacitor)
void tools::smoothExp(double &x, const double &val, const uint32_t &sampleTime, const uint32_t &timeConst) {

  double weight = 1 - exp(-sampleTime/(double)timeConst);
  x = (1-weight)*x + weight*val;
}


// Hysteresis (simulates Inverting Schmitt Trigger)
bool tools::trigger(bool &trigger, const double &val, const float &low, const float &high) {

  bool trigger_last = trigger;

  if (val > high) { trigger = true;  } else
  if (val < low ) { trigger = false; }

  if (trigger_last != trigger) { return true; } else { return false; }
}