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


// Radix-2 Fast Fourier Transform of (complex) sample input
FFT_t tools::FFT(FFT_t samples) {

  const uint16_t N     = samples.size();
  const uint16_t nBits = tools::log2(N);

  // Reversed-bit indexing
  for (uint16_t i = 0; i < N; i++) {

    uint16_t rev = reverseBits(i, nBits);
    if (i < rev) { std::swap(samples[i], samples[rev]); }
  }

  // Perform FFT (http://www.librow.com/articles/article-10)
  for (uint16_t step = 1; step < N; step <<= 1)	{

    const uint32_t jump  = step << 1;
    const double   delta = M_PI / (double)step;
    const double   sine  = sin(delta*0.5);
    const complex  mult  = complex(-2*sine*sine, sin(delta));

    complex factor = complex(1);

    for (uint16_t group = 0; group < step; group++) {

      for (uint32_t pair = group; pair < N; pair += jump) {

        const uint32_t match   = pair + step;
        const complex  product = complex(factor*samples[match]);

        samples[match] = samples[pair] - product;
        samples[pair] += product;
      }

      factor = mult*factor + factor;
    }
  }

  return samples;
}
