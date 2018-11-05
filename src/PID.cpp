#include "PID.h"


PID::PID() {

}


void PID::update(double value, double value_set) {

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
  uint32_t t = micros();
  _dt        = (uint32_t)(t - _timeLast)/1000000.0;
  _timeLast  = t;

  // de(t)/dt
  _errorDiff = (value - _valueLast)/_dt;
  _valueLast =  value;

  // ∫e(t)dt
  if (_error  <= 10) {        // only integrate shortly before reaching e(t) = 0
    _errorInt += _error*_dt;
    _errorInt  = constrain(_errorInt, 0, 1/_i);
  }
}


double PID::getOutput() const {

  // u(t) = P*ΔT + I*e_past - D*dT/dt
  return constrain(_p*_error + _i*_errorInt - _d*_errorDiff, 0, 1);
}


void PID::autotune() {

  // TODO: Autotune algorithm
}