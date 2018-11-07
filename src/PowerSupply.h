//############################################################################//
//                                                                            //
//     POWERSUPPLY.H - Library for operating custon built power supply based  //
//                     on the ESP8266                                         //
//----------------------------------------------------------------------------//
//                   © 2018, Jan Post                                         //
//                                                                            //
//############################################################################//


#ifndef POWERSUPPLY_H
#define POWERSUPPLY_H


//############################################################################//
//                                  HEADERS                                   //
//############################################################################//
//----------------------------------------------------------------------------//
  #include "Arduino.h"                                                        //
//----------------------------------------------------------------------------//
  #include <vector>                                                           //
//----------------------------------------------------------------------------//
  #include "img/splash.h"                                                     //
  #include "timing/Stopwatch.h"                                               //
  #include "timing/Scheduler.h"                                               //
  #include "regulation/Sensor.h"                                              //
  #include "regulation/PID.h"                                                 //
  #include "regulation/DAC.h"                                                 //
  #include "controls/Controls.h"                                              //
//----------------------------------------------------------------------------//
//############################################################################//


//############################################################################//
//                                  DEFINES                                   //
//############################################################################//
//----------------------------------------------------------------------------//
  #define V_FIRMWARE_VERSION     "0.1-a"                                      //
//----------------------------------------------------------------------------//
  #define HEATER_TCR              SS316                                       //
//----------------------------------------------------------------------------//
    #define SS316                 0.000915                                    //
    #define SS316L                0.00092                                     //
    #define SS317                 0.000875                                    //
    #define SS317L                0.00088                                     //
    #define SS304                 0.00105                                     //
    #define SS410                 0.00155                                     //
//----------------------------------------------------------------------------//
  #define PID_P                   1.0                                         //
  #define PID_I                   0.1                                         //
  #define PID_D                   0.08                                        //
//----------------------------------------------------------------------------//
  #ifndef HEATER_RES20                                                        //
    #define HEATER_RES20          6.77        // Resistance of heater at 20°C //
  #endif                                                                      //
//                                                                            //
  #ifndef HEATER_RESCABLE                                                     //
    #define HEATER_RESCABLE       0.27        // Resistance of cable          //
  #endif                                                                      //
//                                                                            //
  #define HEATER_TEMP_MIN         0           // !<0 because Temp is unsigned //
  #define HEATER_TEMP_MAX       230                                           //
//                                                                            //
  #define HEATER_POWER_MIN        0           // !<0 because Power is unsignd //
  #define HEATER_POWER_MAX       40                                           //
//----------------------------------------------------------------------------//
//############################################################################//


//############################################################################//
//                                 TYPEDEFS                                   //
//############################################################################//
//----------------------------------------------------------------------------//
  typedef void(*fptr_t)(void);                                                //
//----------------------------------------------------------------------------//
  enum op_t {                                                                 //
    VOLTAGE_MODE,                                                             //
    CURRENT_MODE,                                                             //
    POWER_MODE,                                                               //
    TEMP_MODE                                                                 //
  };                                                                          //
//----------------------------------------------------------------------------//
//############################################################################//




// =================================== TOOLS ===================================

namespace Tools {

  // Exponential smoothing
  void smoothExp(double&, double, float);
  void smoothExp(double&, double, uint32_t, uint32_t);

  // Inverted Schmitt Trigger
  bool trigger(bool&, double, float, float);

  // Set ESP8266 pins HIGH or LOW very fast
  void digitalWriteFast(uint8_t, bool);

  // Constrains (and modifies) a value according to lower and upper limit
  template <typename T, typename U> void trim(T&, U, U);
}

// ----------------------------------- TOOLS -----------------------------------




// ================================ POWERSUPPLY ================================

class PowerSupply {

protected:
  bool     _running;
  op_t     _mode;
  uint16_t _voltage_set, _current_set, _power_set;

public:
  static Sensor_Load sensor;
  static PID         pid;
  static DAC         dac;
  
  static double voltage, current, power;
  
  PowerSupply(void);
  
  virtual void update(void);
  virtual void regulate(double, double);

  void on    (void) { _running = true;      }
  void off   (void) { _running = false;     }
  void toggle(void) { _running = !_running; }

  void setMode   (op_t     m) { _mode        = m; }
  void setVoltage(uint16_t v) { _voltage_set = v; }
  void setCurrent(uint16_t c) { _current_set = c; }
  void setPower  (uint16_t p) { _power_set   = p; }
};

// -------------------------------- POWERSUPPLY --------------------------------




// =================================== HEATER ==================================

class Heater : public PowerSupply {

protected:
  double   _TCR, _res20, _resCable;
  uint16_t _temperature_set = 200;

public:
  static double resistance, temperature;

  Heater(void);
  
  void update   (void) override; 
  void regulate (void);
  void calibrate(void);

  void setRes20   (double res) { _res20    = res; }
  void setResCable(double res) { _resCable = res; }
  void setTCR     (double tcr) { _TCR      = tcr; }

  void setTemp  (uint16_t t) { _temperature_set = t; }
  void increment(int16_t);
};

// ----------------------------------- HEATER ----------------------------------


#endif // POWERSUPPLY_H
