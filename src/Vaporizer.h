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
  #define BUTTON_DEBOUNCE_TIME   50           // Debounce interval in ms      //
//----------------------------------------------------------------------------//
//############################################################################//




typedef void(*fptr_t)(void);

enum op_t  { TEMP_MODE, POWER_MODE };
enum cmd_t { IDLE, UP, DOWN, ENTER, ENTER2 };




namespace Tools {

  void smoothExp(double &, double, uint32_t, uint32_t);
  bool trigger  (bool &,   double, float,    float);
}




// ================================= STOPWATCH =================================

struct Stopwatch {

 private:

  uint32_t _time, _tick;

 public:

  static uint32_t lifetime;

  Stopwatch (void) { _time    = micros();         }
  ~Stopwatch(void) { lifetime = micros() - _time; }

  void     reset  (void)       { _time = micros();        }
  uint32_t getTime(void) const { return micros() - _time; }

  void     tick   (void)       { _tick++;      }
  uint32_t getTick(void) const { return _tick; }
};

// --------------------------------- STOPWATCH ---------------------------------




// ================================= SCHEDULER =================================

struct Task {

  uint32_t lastExecute, deltaTime;
  fptr_t   execute;

  Task(fptr_t, float);
};

struct Scheduler {

 private:

  static vector<Task> _tasks;
  static bool         _running;
  static float        _tickrate;
  static uint32_t     _lastTick, _lastWait, _lastWaitUntil;

 public:

  Scheduler(void);

  static void add (fptr_t, float);
  static void run (void);
  static void stop(void) { _running = false; };

  static void wait         (uint32_t);
  static void waitUntil    (uint32_t);
  static void forceTickRate(float);

  static uint32_t getTickRate(void) { return _tickrate; }
};

// --------------------------------- SCHEDULER ---------------------------------




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




// =================================== INPUT ===================================

struct Input {

 protected:

  uint8_t _state, _pin;

 public:

  enum   state_t { UP, DOWN };
  fptr_t command;

  uint8_t getPin(void) const { return _pin; }

  virtual uint8_t read(void) = 0;
};


struct Encoder : public Input {

 protected:

  static const uint8_t _stateMachine[7][4];
  uint8_t              _pin2;

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

 protected:

  uint32_t _lastRead;

 public:

  Button(uint8_t, fptr_t);

  uint8_t read(void);
};


struct Switch : public Input {

 private:

  uint32_t _lastRead;
  bool    *_ptr = NULL;

 public:

  Switch(uint8_t);

  void attach(bool* var) { _ptr = var;  }
  void detach(void)      { _ptr = NULL; }

  uint8_t read(void);
};

// ----------------------------------- INPUT -----------------------------------




// ================================= CONTROLS ==================================

class Ctrl {

 private:

  static vector<Encoder> _encoders;
  static vector<Button>  _buttons;
  static vector<Switch>  _switches;

  static vector<fptr_t>  _commands;

  static void _ISR_encoder(void);
  static void _ISR_button (void);

 public:

  Ctrl(void);

  static void add(Encoder);
  static void add(Button);
  static void add(Switch);

  static void update(void);
};

// --------------------------------- CONTROLS ----------------------------------




class GUI {

 protected:



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

  Stopwatch watch;
  Scheduler scheduler;
  Heater    heater;
  Ctrl      controls;
  GUI       gui;

  Vaporizer(void);

  void begin(uint8_t, uint8_t);
  void run  (uint8_t);
  void run  (void) { run(30); }

  static String version(void) { return V_FIRMWARE_VERSION; }
};

// --------------------------------- VAPORIZER ---------------------------------




#endif // VAPORIZER_H
