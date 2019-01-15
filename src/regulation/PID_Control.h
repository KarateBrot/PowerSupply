#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <cstdint>
#include <vector>


class PID_Control {

protected:
  double _p, _error, _i, _errorInt, _d, _errorDiff;
  double _dt, _timeLast, _valueLast;

public:
  PID_Control(void);
  PID_Control(double, double, double);

  void   update   (double, double, uint32_t);
  double getOutput(void) const;
  void   autotune (void);

  void setPID(double p, double i, double d) { _p = p; _i = i; _d = d; }
  PID_Control& setP(double p) { _p = p; return *this; }
  PID_Control& setI(double i) { _i = i; return *this; }
  PID_Control& setD(double d) { _d = d; return *this; }

  std::vector<double> getPID(void) const { return {_p, _i, _d}; }

  //TODO: Implement functions to quantify control performance (L1/L2-Standard)
};


#endif // PID_CONTROL_H
