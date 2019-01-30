#ifndef __TOOLS__POWERSUPPLY_H
#define __TOOLS__POWERSUPPLY_H

#include <cstdint>
#include <climits>
#include <cmath>
#include <vector>
#include <complex>

typedef std::vector<std::complex<double>> FFT_t;



namespace tools {

  // Exponential smoothing
  void smoothExp(double &x, const double &val, const float &weight);
  void smoothExp(double &x, const double &val, const uint32_t &sampleTime, const uint32_t &timeConst);

  // Inverted Schmitt Trigger
  bool trigger(bool &trigger, const double &val, const float &low, const float &high);

  // Fast Fourier Transform of (complex) sample input
  FFT_t FFT(const FFT_t &samples); // TODO: More functionality

  // Confines a value between lower and upper limit
  template <typename T, typename U>
  T limit(const T &val, const U &low, const U &high) {
    if (val > high) { return (T)high; } else
    if (val < low ) { return (T)low;  } else
    return val;
  }

  // Confines (and modifies!) a value according to lower and upper limit
  template <typename T, typename U>
  bool trim(T &val, const U &low, const U &high) {
    if (val > high) { val = high; return true; } else
    if (val < low ) { val = low;  return true; } else
    return false;
  }

  // Reverses the first n bits of a value with type T
  template <typename T>
  T reverseBits(T val, uint8_t n = sizeof(T)*CHAR_BIT) {
    T temp = 0;
    for (size_t i = 0; i < n; i++, val >>= 1) {
      temp = (temp << 1) | (val & 1);
    }
    return temp;
  }
}



#endif // __TOOLS__POWERSUPPLY_H


