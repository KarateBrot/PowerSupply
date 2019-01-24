#ifndef __TOOLS__POWERSUPPLY_H
#define __TOOLS__POWERSUPPLY_H

#include <cstdint>
#include <cmath>



namespace Tools {

  // Exponential smoothing
  void smoothExp(double &x, const double &val, const float &weight);
  void smoothExp(double &x, const double &val, const uint32_t &sampleTime, const uint32_t &timeConst);

  // Inverted Schmitt Trigger
  bool trigger(bool &trigger, const double &val, const float &low, const float &high);

  // Confines a value between lower and upper limit
  template<typename T, typename U> 
  T limit(const T &val, const U &low, const U &high);

  // Confines (and modifies!) a value according to lower and upper limit
  template<typename T, typename U> 
  bool trim(T &val, const U &low, const U &high);
}



#endif // __TOOLS__POWERSUPPLY_H


