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
  #include <Adafruit_INA219.h>                // Current & voltage sensor     //
  #include <Adafruit_MCP4725.h>               // Digital to analog converter  //
  #include <Adafruit_SSD1306.h>               // Display driver               //
  #include <Adafruit_GFX.h>                   // Graphics engine              //
//----------------------------------------------------------------------------//
  #include <vector>                                                           //
//----------------------------------------------------------------------------//
  #include "Stopwatch.h"                                                      //
  #include "Scheduler.h"                                                      //
  #include "img/splash.h"                                                     //
//----------------------------------------------------------------------------//
//############################################################################//


//############################################################################//
//                                 SETTINGS                                   //
//############################################################################//
//----------------------------------------------------------------------------//
  #define V_FIRMWARE_VERSION     "0.1-a"                                      //
//----------------------------------------------------------------------------//
  #define WIRE_FREQ               800000L                                     //
  #define PWM_RANGE               1023                                        //
  #define PWM_FREQ                4000        // Max: CPU_CLOCK / PWM_RANGE   //
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
  #define BUTTON_DEBOUNCE_TIME   50           // Debounce interval in ms      //
//----------------------------------------------------------------------------//
//############################################################################//




typedef void(*fptr_t)(void);

enum op_t  { TEMP_MODE, POWER_MODE };

namespace Tools {

  void smoothExp(double &, double, uint32_t, uint32_t); // Exponential moving average
  bool trigger  (bool &,   double, float,    float);    // Inverted Schmitt Trigger

  template <typename T, typename U> void trim(T &, U, U);
}




// ================================== SENSORS ==================================

struct Sensor {


};


struct Sensor_Power : public Sensor {

 private:

  static Adafruit_INA219 _INA219;

 public:

  Sensor_Power(void);

  void setPrecision(bool);

  double getCurrent(void);
  double getVoltage(void);
};


struct Sensor_Ambient : public Sensor {

 private:

 public:

  void read(void);
};

// ---------------------------------- SENSORS ----------------------------------




// ==================================== DAC ====================================

struct DAC {

 private:

  static Adafruit_MCP4725 _MCP4725;

 public:

  DAC(void);

  void setOutput(uint16_t);
};

// ------------------------------------ DAC ------------------------------------




// ==================================== PID ====================================

class PID_Ctrl {

  double _p, _error, _i, _errorInt, _d, _errorDiff;
  double _dt, _timeLast, _valueLast;
  DAC   *_dacPtr;

  void   _update(double, double);

 public:

  PID_Ctrl(void);

  void attach(DAC* d) { _dacPtr = d;    };
  void detach(void)   { _dacPtr = nullptr; };

  void setPID(double p, double i, double d) { _p = p; _i = i; _d = d; }
  PID_Ctrl& setP(double p) { _p = p; return *this; }
  PID_Ctrl& setI(double i) { _i = i; return *this; }
  PID_Ctrl& setD(double d) { _d = d; return *this; }

  std::vector<double> getPID(void) const;

  //TODO: Implement functions to quantify control performance (L1/L2-Standard)

  double getOutput(void) const;
  void   regulate (double, double);
  void   autotune (void);
};

// ------------------------------------ PID ------------------------------------




// =================================== HEATER ==================================

class Heater {

  bool     _running;
  double   _TCR, _res20, _resCable;
  op_t     _mode = TEMP_MODE;
  uint16_t _power_set = 10, _temperature_set = 200;

 public:

  Sensor_Power sensor;
  DAC          dac;
  PID_Ctrl     pid;

  double voltage, current, resistance, power, temperature;

  Heater(void);

  void setRes20   (double res) { _res20    = res; }
  void setResCable(double res) { _resCable = res; }
  void setTCR     (double tcr) { _TCR      = tcr; }

  void setMode  (op_t     m) { _mode            = m; }
  void setTemp  (uint16_t t) { _temperature_set = t; }
  void setPower (uint16_t p) { _power_set       = p; }
  void increment(int16_t);

  void on    (void) { _running = true;      }
  void off   (void) { _running = false;     }
  void toggle(void) { _running = !_running; }

  void update   (void);
  void regulate (void);
  void calibrate(void);
};

// ----------------------------------- HEATER ----------------------------------




// =================================== INPUT ===================================

struct Input {

  friend class Controls;

 protected:

  uint8_t _state, _pin;

 public:

  enum   state_t { UP, DOWN };
  fptr_t command;

  uint8_t getPin(void) const { return _pin; }

  virtual uint8_t read(void) = 0;
};


struct Encoder : public Input {

  friend class Controls;

 private:

  static std::vector<Encoder> _buffer;
  static const uint8_t        _stateMachine[7][4];
  uint8_t                     _pin2;

  static void _ISR(void);

 public:

  enum state_t {
    START, CW_FINAL, CW_BEGIN, CW_NEXT, CCW_BEGIN, CCW_FINAL, CCW_NEXT,
    CW = 0x10, CCW = 0x20
  };

  fptr_t commandCCW;

  Encoder(uint8_t, uint8_t, fptr_t, fptr_t);

  uint8_t getPin2(void) const { return _pin2; }
  uint8_t read(void);
};


struct Button : public Input {

  friend class Controls;

 private:

  static std::vector<Button> _buffer;
  uint32_t                   _lastRead;

  static void _ISR(void);

 public:

  Button(uint8_t, fptr_t);

  uint8_t read(void);
};


struct Switch : public Input {

  friend class Controls;

 private:

  static std::vector<Switch> _buffer;
  bool                      *_ptr;
  uint32_t                   _lastRead;

 public:

  Switch(uint8_t, bool*);

  uint8_t read(void);
};

// ----------------------------------- INPUT -----------------------------------




// ================================= CONTROLS ==================================

class Controls {

  friend class Encoder;
  friend class Button;
  friend class Switch;

 private:

  static std::vector<fptr_t> _commands;

 public:

  Controls(void);

  static void add(Encoder);
  static void add(Button);
  static void add(Switch);

  static void update(void);
};

// --------------------------------- CONTROLS ----------------------------------




// ================================= VAPORIZER =================================

class Vaporizer {

 public:

  static Scheduler scheduler;

  Stopwatch watch;
  Heater    heater;
  Controls  controls;

  Vaporizer(void);

  void begin(uint8_t, uint8_t);
  void run  (uint8_t);
  void run  (void) { run(30); }

  static String version(void) { return V_FIRMWARE_VERSION; }
};

// --------------------------------- VAPORIZER ---------------------------------




#endif // VAPORIZER_H
