#ifndef PID_H
#define PID_H

#include <cstdint>
#include <vector>


class PID {

protected:
  double _p, _error, _i, _errorInt, _d, _errorDiff;
  double _dt, _timeLast, _valueLast;

public:
  PID(void);
  PID(double, double, double);

  void   update   (double, double, uint32_t);
  double getOutput(void) const;
  void   autotune (void);

  void setPID(double p, double i, double d) { _p = p; _i = i; _d = d; }
  PID& setP(double p) { _p = p; return *this; }
  PID& setI(double i) { _i = i; return *this; }
  PID& setD(double d) { _d = d; return *this; }

  std::vector<double> getPID(void) const { return {_p, _i, _d}; }

  //TODO: Implement functions to quantify control performance (L1/L2-Standard)
};


#endif // PID_H
