#include "FFT.h"


// Radix-2 Fast Fourier Transform of (complex) sample input
complexList FFT::process(complexList samples) {

  const uint16_t N     = samples.size();
  const uint16_t nBits = tools::log2(N);

  if (!tools::isPower2(N)) return complexList();

  // Reversed-bit indexing
  for (uint16_t i = 0; i < N; i++) {

    uint16_t rev = tools::reverseBits(i, nBits);
    if (i < rev) { std::swap(samples[i], samples[rev]); }
  }

  // Perform FFT (http://www.librow.com/articles/article-10)
  for (uint16_t step = 1; step < N; step <<= 1)	{

    const uint32_t jump  = step << 1;
    const float    delta = (float)M_PI / (float)step;
    const float    sine  = sin(delta*0.5f);
    const complex  mult  = complex(-2*sine*sine, sin(delta));

    complex factor = complex(1.0f);

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


// Find peaks of FFT spectrum. Higher sensitivity yields more peaks
std::vector<uint16_t> FFT::findPeaks(const complexList &data, const float &sens) {

  typedef std::vector<float>    fList;
  typedef std::vector<uint16_t> iList;

  const uint16_t N = data.size();
  const uint16_t M = N/2;

  if (!M) return iList();

  fList  mag  = fList(M);
  fList  diff = fList(M);
  float  rms  = 0.0f;
  iList  peaks;

  mag[0]  = std::abs(data[0]);
  diff[0] = 0.0f;

  for(uint16_t i = 1; i < M; i++) {

    mag[i]  = std::abs(data[i]);
    diff[i] = mag[i] - mag[i-1];
    rms    += pow(mag[i], 2);
  }
  rms /= (float)M;
  rms  = sqrt(rms);

  // Search for peaks
  for(uint16_t i = 0; i < M-1; i++) {

    const bool zeroTransit = tools::sgn(diff[i]) > 0 && tools::sgn(diff[i+1]) < 0;
    const bool significant = mag[i] > 2*rms/sens;

    if (zeroTransit && significant) { 
      peaks.emplace_back(i); 
    }
  }

  return peaks;
}