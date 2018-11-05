//############################################################################//
//                                                                            //
//     VAPORIZER.H - Library for operating custon built vaporizer based       //
//                   on ESP8266                                               //
//----------------------------------------------------------------------------//
//                   © 2018, Jan Post                                         //
//                                                                            //
//############################################################################//


#ifndef VAPORIZER_H
#define VAPORIZER_H


//############################################################################//
//                                 LIBRARIES                                  //
//############################################################################//
//----------------------------------------------------------------------------//
  #include <Arduino.h>                                                        //
//----------------------------------------------------------------------------//
  #include <vector>                                                           //
//----------------------------------------------------------------------------//
  #include "Stopwatch.h"                                                      //
  #include "Scheduler.h"                                                      //
  #include "Sensor.h"                                                         //
  #include "DAC.h"                                                            //
  #include "PID.h"                                                            //
  #include "Controls.h"                                                       //
  #include "img/splash.h"                                                     //
//----------------------------------------------------------------------------//
//############################################################################//


//############################################################################//
//                                 SETTINGS                                   //
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


typedef void(*fptr_t)(void);

enum op_t  { TEMP_MODE, POWER_MODE };




// =================================== TOOLS ===================================

namespace Tools {

  void smoothExp(double &, double, uint32_t, uint32_t); // Exponential moving average
  bool trigger  (bool &,   double, float,    float);    // Inverted Schmitt Trigger

  void digitalWriteFast(uint8_t, bool);  // Set ESP8266 pins HIGH or LOW very fast

  template <typename T, typename U> void trim(T &, U, U);
}

// ----------------------------------- TOOLS -----------------------------------




// ================================= REGULATOR =================================

class Regulator {

protected:
  bool     _running;
  uint16_t _voltage_set, _current_set, _power_set;

public:
  static Sensor_Load sensor;
  static PID         pid;
  static DAC         dac;
  
  static double voltage, current, power;
  
  Regulator(void);
  
  static void update  (void);
  static void regulate(double, double);

  void on    (void) { _running = true;      }
  void off   (void) { _running = false;     }
  void toggle(void) { _running = !_running; }

  void setVoltage(uint16_t v) { _voltage_set = v; }
  void setCurrent(uint16_t c) { _current_set = c; }
  void setPower  (uint16_t p) { _power_set   = p; }
};

// --------------------------------- REGULATOR ---------------------------------




// =================================== HEATER ==================================

class Heater : public Regulator {

protected:
  double   _TCR, _res20, _resCable;
  op_t     _mode = TEMP_MODE;
  uint16_t _temperature_set = 200;

public:
  static double resistance, temperature;

  Heater(void);
  
  void update   (void); 
  void regulate (void);
  void calibrate(void);

  void setRes20   (double res) { _res20    = res; }
  void setResCable(double res) { _resCable = res; }
  void setTCR     (double tcr) { _TCR      = tcr; }

  void setMode  (op_t     m) { _mode            = m; }
  void setTemp  (uint16_t t) { _temperature_set = t; }
  void increment(int16_t);
};

// ----------------------------------- HEATER ----------------------------------


#endif // VAPORIZER_H
