#ifndef FFT_H
#define FFT_H

#include <complex>
#include <vector>

#include "tools/tools.h"

typedef std::complex<float>  complex;
typedef std::vector<complex> complexList;



struct FFT {
  
  static complexList process(complexList);
  static std::vector<uint16_t> findPeaks(const complexList&, const float& = 1.0f);
};



#endif // FFT_H