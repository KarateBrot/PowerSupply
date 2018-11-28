#include "PID.h"


PID::PID() {
  
}


PID::PID(double p, double i, double d) : 

  _p(p), _i(i), _d(d) {
}


void PID::update(double value, double value_set, uint32_t timeCurrent) {

// -------------------------------------------------------------------------- //
//                          --< PID-CONTROLLER >--                            //
// -----------------:-------------------------------------------------------- //
// SIGNAL OUTPUT:   : u(t) = P*e(t) + I*∫e(t)dt + D*de(t)/dt                  //
// -----------------:-------------------------------------------------------- //
// ERROR:           :     e(t)     = T_set - T(t) = ΔT                        //
// PAST ERR:        :     ∫e(t)dt  = e_past                                   //
// PREDICTED ERR:   :     de(t)/dt = -dT/dt                                   //
// -----------------:-------------------------------------------------------- //
//       ──►        : u(t) = P*ΔT + I*e_past - D*dT/dt                        //
// -----------------:-------------------------------------------------------- //

  // e(t)
  _error = value_set - value;

  // dt
  _dt       = (uint32_t)(timeCurrent - _timeLast)/1000000.0;
  _timeLast = timeCurrent;

  // de(t)/dt
  _errorDiff = (value - _valueLast)/_dt;
  _valueLast =  value;

  // ∫e(t)dt
  if (_error <= 10) {        // only integrate shortly before reaching e(t) = 0
    
    _errorInt += _error*_dt;
    
    // Constrain _errorInt
    if (_errorInt < 0)      { _errorInt = 0;      } else
    if (_errorInt > 1.0/_i) { _errorInt = 1.0/_i; }
  }
}


double PID::getOutput() const {

  // u(t) = P*e(t) + I*∫e(t)dt - D*dT/dt
  double u = _p*_error + _i*_errorInt - _d*_errorDiff;

  // Constrain output signal
  if (u < 0.0) { u = 0.0; } else
  if (u > 1.0) { u = 1.0; }

  return u;
}


void PID::autotune() {

  // TODO: Autotune algorithm
}