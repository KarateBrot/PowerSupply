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
  #include <Adafruit_GFX.h>                   // Graphics engine              //
  #include <Adafruit_SSD1306.h>               // Display driver               //
//----------------------------------------------------------------------------//
  #include <vector>                                                           //
    using namespace std;                                                      //
//----------------------------------------------------------------------------//
  #include <img/splash.h>                                                     //
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
  #define PWM_FREQ                4000        // Max: CPU_CLOCK / PWM_RANGE   //                             //
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
//----------------------------------------------------------------------------//
//############################################################################//




typedef void(*fptr_t)(void);

enum op_t  { TEMP_MODE, POWER_MODE };
enum cmd_t { IDLE, UP, DOWN, ENTER, ENTER2 };




// =================================== TIMER ===================================

struct Stopwatch {

 private:

  uint32_t _time;

 public:

  static uint32_t lifetime;

  Stopwatch (void) { _time    = micros();         }
  ~Stopwatch(void) { lifetime = micros() - _time; }

  void     reset  (void)       { _time = micros();        }
  uint32_t getTime(void) const { return micros() - _time; }
};

struct Task {

  float    tickRate;
  uint32_t lastExecute;
  fptr_t   execute;

  Task(fptr_t, float);
};

struct Timer {

 private:

  uint32_t _tick;

  static vector<Task> _tasks;
  static float        _tickrate;
  static uint32_t     _lastTick, _lastWait, _deltaTime;
  static bool         _running;

 public:

  Timer(void);

  void     tick   (void)       { _tick++;      }
  uint32_t getTick(void) const { return _tick; }

  static void add (fptr_t, float);
  static void run (void);
  static void stop(void) { _running = false; };

  static void     waitUntil    (uint32_t);
  static void     forceTickRate(float);
  static uint32_t getTickRate  (void) { return _tickrate; }
};

// ----------------------------------- TIMER -----------------------------------




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
  DAC   *_dacPtr = NULL;

  void   _update(double, double);

 public:

  PID_Ctrl(void);

  PID_Ctrl& attach(DAC* d) { _dacPtr = d;    return *this; };
  PID_Ctrl& detach(void)   { _dacPtr = NULL; return *this; };

  PID_Ctrl& setP  (double p) { _p = p; return *this; }
  PID_Ctrl& setI  (double i) { _i = i; return *this; }
  PID_Ctrl& setD  (double d) { _d = d; return *this; }
  PID_Ctrl& setPID(double p, double i, double d) { _p = p; _i = i; _d = d; return *this; }

  vector<double> getPID(void) const;

  //TODO: Implement functions to quantify control performance (L1/L2-Standard)

  double getOutput(void) const;
  void   regulate (double, double);
  void   autotune (void);
};

// ------------------------------------ PID ------------------------------------




// =================================== HEATER ==================================

class Heater {

  double _TCR, _res20, _resCable;
  bool   _running = false;
  op_t   _mode    = TEMP_MODE;

 public:

  Sensor_Power sensor;
  DAC          dac;
  PID_Ctrl     pid;

  double   voltage, current, resistance, power, temperature;
  uint16_t power_set = 10, temperature_set = 200;

  Heater(void);

  Heater& setRes20   (double res) { _res20    = res; return *this; }
  Heater& setResCable(double res) { _resCable = res; return *this; }
  Heater& setTCR     (double tcr) { _TCR      = tcr; return *this; }

  Heater& setTemp (uint16_t t) { temperature_set = t; return *this; }
  Heater& setPower(uint16_t p) { power_set       = p; return *this; }

  Heater& setMode(op_t m) { _mode = m; return *this; }

  Heater& on    (void) { _running = true;      return *this; }
  Heater& off   (void) { _running = false;     return *this; }
  Heater& toggle(void) { _running = !_running; return *this; }

  void update   (void);
  void regulate (void);
  void calibrate(void);
};

// ----------------------------------- HEATER ----------------------------------




// ================================== CONTROLS =================================

struct Controls {

 protected:

  uint8_t _state, _pin;
  cmd_t   _command;

 public:

  enum state_t { UP, DOWN };

  uint8_t getPin    (void) const { return _pin;     }
  cmd_t   getCommand(void) const { return _command; }

  virtual uint8_t read(void) = 0;
};


struct Encoder : public Controls {

 private:

  static const uint8_t _stateMachine[7][4];
  uint8_t              _pin2;
  cmd_t                _commandCCW;

 public:

  enum state_t {

    START, CW_FINAL, CW_BEGIN, CW_NEXT, CCW_BEGIN, CCW_FINAL, CCW_NEXT,
    CW = 0x10, CCW = 0x20
  };

  Encoder(uint8_t, uint8_t, cmd_t, cmd_t);

  uint8_t getPin2      (void) const { return _pin2;       }
  cmd_t   getCommandCCW(void) const { return _commandCCW; }

  uint8_t read(void);
};


struct Button : public Controls {

 private:

  uint32_t _lastRead;

 public:

  Button(uint8_t, cmd_t);

  uint8_t read(void);
};


struct Switch : public Controls {

 private:

  uint32_t _lastRead;
  bool    *_ptr = NULL;

 public:

  Switch(uint8_t);

  void attach(bool* var) { _ptr = var;  }
  void detach(void)      { _ptr = NULL; }

  uint8_t read(void);
};

// ---------------------------------- CONTROLS ---------------------------------




// =================================== INPUT ===================================

class Input {

 private:

  static vector<Encoder> _encoders;
  static vector<Button>  _buttons;
  static vector<Switch>  _switches;

  static vector<cmd_t>   _commands;

  static void _ISR_encoder(void);
  static void _ISR_button (void);

  static void _addCommand   (cmd_t c) { _commands.push_back(c); }
  static void _handleCommand(cmd_t);

 public:

  Input(void);

  static void add(Encoder);
  static void add(Button);
  static void add(Switch);

  static void update(void);
};

// ----------------------------------- INPUT -----------------------------------




class GUI {

 protected:

  static Timer timer;

 public:

  static Adafruit_SSD1306 display;

  GUI(void);

  void draw (void);
  void clear(void);
};




class Settings : public GUI {


};




class Infoscreen : public GUI {


};




// ================================= VAPORIZER =================================

class Vaporizer {

 public:

  Timer  timer;
  Heater heater;
  Input  input;
  GUI    gui;

  Vaporizer(void);

  void begin(uint8_t, uint8_t);
  void run  (uint8_t);

  static String version(void) { return V_FIRMWARE_VERSION; }
};

// --------------------------------- VAPORIZER ---------------------------------




#endif // VAPORIZER_H
