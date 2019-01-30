#include "tools.h"


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


// Fast Fourier Transform of (complex) sample input
FFT_t tools::FFT(const FFT_t &samples) {

  uint16_t N = samples.size();

  if (N <= 1) return samples;

  uint16_t M = N/2;

  FFT_t 
    xEven(M, 0),
    xOdd(M, 0);

  for (size_t i = 0; i < M; i++) {

    xEven[i] = samples[2*i];
    xOdd[i]  = samples[2*i + 1];
  }

  // Recursion too slow. TODO: Use reversed-bit indexing instead
  FFT_t
    fEven = FFT(xEven),
    fOdd  = FFT(xOdd);

  FFT_t bins(N, 0);
  
  for (size_t k = 0; k < M; k++) {

    std::complex<double> t = std::polar(1.0, -2*M_PI*k/N) * fOdd[k];

    bins[k]     = fEven[k] + t;
    bins[k + M] = fEven[k] - t;
  }

  return bins;
}