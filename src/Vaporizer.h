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




<<<<<<< HEAD
enum operation_t { TEMP_MODE, POWER_MODE };
enum state_t     { OFF, ON };
=======
  enum operation_t { TEMP_MODE, POWER_MODE };
  enum state_t     { OFF, ON };
>>>>>>> 60d99b75c278af8afa9b1f92da3ea290bc0e0881




// =================================== TIMER ===================================

struct Timer {

 private:

<<<<<<< HEAD
  uint32_t _tick, _time, _lastWait, _lastTick;
=======
    uint32_t _tick, _time, _lastWait, _lastTick;
>>>>>>> 60d99b75c278af8afa9b1f92da3ea290bc0e0881

 public:

  Timer (void);
  ~Timer(void);

  static uint32_t lifetime;

<<<<<<< HEAD
  void waitUntil(uint32_t);
=======
    void waitUntil(uint32_t);

    void  tickRate (uint8_t);
    float getTickRate(void);

    void     tick   (void)       { _tick++; }
    uint32_t getTick(void) const { return _tick; }

    void     reset  (void)       { _time = micros(); }
    uint32_t getTime(void) const { return micros() - _time; }
  };
>>>>>>> 60d99b75c278af8afa9b1f92da3ea290bc0e0881

  void  tickRate (uint8_t);
  float getTickRate(void);

  void     tick   (void)       { _tick++; }
  uint32_t getTick(void) const { return _tick; }

  void     reset  (void)       { _time = micros(); }
  uint32_t getTime(void) const { return micros() - _time; }
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

<<<<<<< HEAD
  PID_Ctrl& attach(DAC* d) { _dacPtr = d;    return *this; };
  PID_Ctrl& detach(void)   { _dacPtr = NULL; return *this; };
=======
    //TODO: Implement functions to quantify control performance (L1/L2-Standard)
>>>>>>> 60d99b75c278af8afa9b1f92da3ea290bc0e0881

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

 public:

  Sensor_Power sensor;
  DAC          dac;
  PID_Ctrl     pid;

  double   voltage, current, resistance, power, temperature;
  uint16_t power_set = 10, temperature_set = 200;

  state_t     state = OFF;
  operation_t mode  = TEMP_MODE;

  Heater(void);

  Heater& setRes20   (double res) { _res20    = res; return *this; }
  Heater& setResCable(double res) { _resCable = res; return *this; }
  Heater& setTCR     (double tcr) { _TCR      = tcr; return *this; }

  Heater& setTemp (uint16_t t) { temperature_set = t; return *this; }
  Heater& setPower(uint16_t p) { power_set       = p; return *this; }

  Heater& setMode(operation_t m) { mode = m; return *this; }

  Heater& on (void) { state = ON;  return *this; }
  Heater& off(void) { state = OFF; return *this; }

  void update   (void);
  void regulate (void);
  void calibrate(void);
};

// ----------------------------------- HEATER ----------------------------------


<<<<<<< HEAD
=======
    uint8_t  _state, _pin;
    uint32_t _lastRead;
>>>>>>> 60d99b75c278af8afa9b1f92da3ea290bc0e0881


<<<<<<< HEAD
// ================================== CONTROLS =================================

struct Controls {

 protected:
=======
    enum state_t { UP, DOWN };

    virtual uint8_t read(void) = 0;
  };
>>>>>>> 60d99b75c278af8afa9b1f92da3ea290bc0e0881

  uint8_t  _state, _pin;
  uint32_t _lastRead;

 public:

  enum state_t { UP, DOWN };

<<<<<<< HEAD
  virtual uint8_t read(void) = 0;
};
=======
    uint8_t              _pin2;
    static const uint8_t _stateMachine[7][4];
>>>>>>> 60d99b75c278af8afa9b1f92da3ea290bc0e0881


struct Encoder : public Controls {

 private:

<<<<<<< HEAD
  uint8_t              _pin2;
  static const uint8_t _stateMachine[7][4];

 public:

  enum state_t {

    START, CW_FINAL, CW_BEGIN, CW_NEXT, CCW_BEGIN, CCW_FINAL, CCW_NEXT,
    CW = 0x10, CCW = 0x20
=======
    Encoder(uint8_t, uint8_t);

    uint8_t read (void);
>>>>>>> 60d99b75c278af8afa9b1f92da3ea290bc0e0881
  };

  Encoder(uint8_t, uint8_t);

  uint8_t read(void);
};


<<<<<<< HEAD
struct Button : public Controls {

 private:

 public:

  Button(uint8_t);

  uint8_t read(void);
};
=======
   public:

    Button(uint8_t);
>>>>>>> 60d99b75c278af8afa9b1f92da3ea290bc0e0881


struct Switch : public Controls {

 private:

  bool *_ptr = NULL;

<<<<<<< HEAD
 public:
=======
    bool *_ptr = NULL;
>>>>>>> 60d99b75c278af8afa9b1f92da3ea290bc0e0881

  Switch(uint8_t);

<<<<<<< HEAD
  void attach(bool* var) { _ptr = var;  }
  void detach(void)      { _ptr = NULL; }

  uint8_t read(void);
};

=======
    Switch(uint8_t);

    void attach(bool* var) { _ptr = var;  }
    void detach(void)      { _ptr = NULL; }
>>>>>>> 60d99b75c278af8afa9b1f92da3ea290bc0e0881

struct Controls_List {

  vector<Encoder> _encoders;
  vector<Button>  _buttons;
  vector<Switch>  _switches;
};

// ---------------------------------- CONTROLS ---------------------------------




// =================================== INPUT ===================================

class Input {

 protected:

  static Controls_List _controls;

  void _isr(void);

 public:

  Input(void);
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

<<<<<<< HEAD
 public:

  Timer  timer;
  Heater heater;
  Input  input;
  GUI    gui;

  Vaporizer(void);
=======
    Vaporizer(void);

    void begin(uint8_t, uint8_t);
    void run  (uint8_t);
>>>>>>> 60d99b75c278af8afa9b1f92da3ea290bc0e0881

  void begin(uint8_t, uint8_t);
  void run  (uint8_t);

  static String version(void) { return V_FIRMWARE_VERSION; }
};

// --------------------------------- VAPORIZER ---------------------------------




#endif // VAPORIZER_H
