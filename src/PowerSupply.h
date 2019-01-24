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
  #include <cstdint>                                                          //
  #include <cmath>                                                            //
  #include <functional>                                                       //
//----------------------------------------------------------------------------//
  #include "Arduino.h"        // TODO: Make header independent from Arduino.h //
//----------------------------------------------------------------------------//
  #include "tools/Tools.h"                                                    //
  #include "img/splash.h"                                                     //
  #include "timing/Stopwatch.h"                                               //
  #include "timing/Scheduler.h"                                               //
  #include "regulation/PID_Control.h"                                         //
  #include "regulation/Sensor.h"                                              //
  #include "regulation/DAC.h"                                                 //
  #include "ui/CLI.h"                                                         //
  #include "controls/Controls.h"                                              //
//----------------------------------------------------------------------------//
//############################################################################//


//############################################################################//
//                                  DEFINES                                   //
//############################################################################//
//----------------------------------------------------------------------------//
  #define V_FIRMWARE_VERSION    "0.2-a"                                       //
//----------------------------------------------------------------------------//
  #define HEATER_TCR             SS316                                        //
//----------------------------------------------------------------------------//
    #define SS316                0.000915                                     //
    #define SS316L               0.00092                                      //
    #define SS317                0.000875                                     //
    #define SS317L               0.00088                                      //
    #define SS304                0.00105                                      //
    #define SS410                0.00155                                      //
//----------------------------------------------------------------------------//
  #define PID_P                  1.0                                          //
  #define PID_I                  0.1                                          //
  #define PID_D                  0.08                                         //
//----------------------------------------------------------------------------//
  #define VOLTAGE_MIN            0                                            //
  #define VOLTAGE_MAX        10000                                            //
//----------------------------------------------------------------------------//
  #define CURRENT_MIN            0                                            //
  #define CURRENT_MAX         5000                                            //
//----------------------------------------------------------------------------//
  #define POWER_MIN              0            // !<0 because Power is unsignd //
  #define POWER_MAX             40                                            //
//----------------------------------------------------------------------------//
  #define TEMPERATURE_MIN        0            // !<0 because Temp is unsigned //
  #define TEMPERATURE_MAX      230                                            //
//----------------------------------------------------------------------------//
  #define HEATER_RES20           6.77         // Resistance of heater at 20°C //
  #define HEATER_RESCABLE        0.27         // Resistance of cable          //
//----------------------------------------------------------------------------//
//############################################################################//


//############################################################################//
//                                 TYPEDEFS                                   //
//############################################################################//
//----------------------------------------------------------------------------//
  typedef void(*fptr_t)(void);                                                //
//----------------------------------------------------------------------------//
  template<typename T>                                                        //
  using fptr_args_t = std::function<T(void*)>;                                //
//----------------------------------------------------------------------------//
  enum class Mode : uint8_t {                                                 //
    VOLTAGE,                                                                  //
    CURRENT,                                                                  //
    POWER,                                                                    //
    TEMPERATURE                                                               //
  };                                                                          //
//----------------------------------------------------------------------------//
//############################################################################//




// ================================ POWERSUPPLY ================================

class PowerSupply {

protected:
  bool     _running;
  Mode     _mode;
  uint16_t _voltage_set, _current_set, _power_set;

  void _converge(const double &val, const double &val_set);

public:
  PID_Control        pid;
  
  static DAC         dac;
  static Sensor_Load sensor;
  static double      voltage, current, power;
  
  PowerSupply(void);
  
  void on    (void) { _running = true;      }
  void off   (void) { _running = false;     }
  void toggle(void) { _running = !_running; }

  void setMode   (const Mode     &m) { _mode        = m; }
  void setVoltage(const uint16_t &v) { _voltage_set = v; }
  void setCurrent(const uint16_t &c) { _current_set = c; }
  void setPower  (const uint16_t &p) { _power_set   = p; }
  
  virtual void increment(const int16_t &val);
  virtual void update   (void);
  virtual void regulate (void);
};

// -------------------------------- POWERSUPPLY --------------------------------




// =================================== HEATER ==================================

class Heater : public PowerSupply {

private:
  double   _TCR, _res20, _resCable;
  uint16_t _temperature_set = 200;

public:
  static double resistance, temperature;

  Heater(void);
  
  void calibrate(void);
  
  void setRes20   (const double   &res) { _res20           = res; }
  void setResCable(const double   &res) { _resCable        = res; }
  void setTCR     (const double   &tcr) { _TCR             = tcr; }
  void setTemp    (const uint16_t &t  ) { _temperature_set = t;   }

  void increment(const int16_t &val) override;
  void update   (void)               override; 
  void regulate (void)               override;
};

// ----------------------------------- HEATER ----------------------------------


#endif // POWERSUPPLY_H
