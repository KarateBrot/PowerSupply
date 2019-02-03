#ifndef __TOOLS__POWERSUPPLY_H
#define __TOOLS__POWERSUPPLY_H

#include <cstdint>
#include <climits>
#include <cmath>
#include <vector>
#include <complex>

typedef std::complex<double> complex;
typedef std::vector<complex> FFT_t;



namespace tools {

  // Check if value is a power of 2
  bool isPower2(const uint32_t& val);
  
  // log2 for integers with complexity O(N)
  uint8_t log2(uint32_t val);

  // Exponential smoothing
  void smoothExp(double &x, const double &val, const float &weight);
  void smoothExp(double &x, const double &val, const uint32_t &sampleTime, const uint32_t &timeConst);

  // Inverted Schmitt Trigger
  bool trigger(bool &trigger, const double &val, const float &low, const float &high);

  // Radix-2 Fast Fourier Transform of (complex) sample input
  FFT_t FFT(FFT_t samples); // TODO: More functionality

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
    if (val > high) { val = (T)high; return true; } else
    if (val < low ) { val = (T)low;  return true; } else
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


