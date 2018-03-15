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
  #include <Adafruit_INA219.h>                                                //
  #include <Adafruit_MCP4725.h>                                               //
  #include <Adafruit_GFX.h>                                                   //
  #include <Adafruit_SSD1306.h>                                               //
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
  #define HEATER_TCR              SS316                            // at 20°C //
//----------------------------------------------------------------------------//
    #define SS316                 0.000915                                    //
    #define SS316L                0.00092                                     //
    #define SS317                 0.000875                                    //
    #define SS317L                0.00088                                     //
    #define SS304                 0.00105                                     //
    #define SS410                 0.00155                                     //
//----------------------------------------------------------------------------//
  #define HEATER_RES20            6.77                                        //
  #define HEATER_RESCABLE         0.27                                        //
//----------------------------------------------------------------------------//
  #define PID_P                   1.0                                         //
  #define PID_I                   0.1                                         //
  #define PID_D                   0.08                                        //
//----------------------------------------------------------------------------//
//############################################################################//




namespace Vaporizer {




  enum mode_t  { TEMP_MODE, POWER_MODE };
  enum state_t { OFF, ON };




  // ================================== TIMER ==================================

  struct Timer {

   private:

    uint32_t _time, _lastWaitCall, _lastCycle;

   public:

    Timer(void);

    uint32_t counter = 0;

    void     startTime (void)       { _time = micros(); }
    uint32_t getTime   (void) const { return micros() - _time; }
    void     waitUntil (uint32_t);
    void     limitCPS  (uint8_t);
    float    getCPS    (void);
  };

  // ---------------------------------- TIMER ----------------------------------




  // ================================= SENSORS =================================

  struct Sensor {

    virtual void read(void) = 0;
  };


  struct Sensor_Power : public Sensor {

   private:

    static Adafruit_INA219 _INA219;

   public:

    double current, voltage;

    Sensor_Power(void);

    void setPrecision(bool);
    void read        (void);
  };


  struct Sensor_Ambient : public Sensor {

   private:

   public:

    void read(void);
  };

  // --------------------------------- SENSORS ---------------------------------




  // =================================== DAC ===================================

  struct DAC {

   private:

    static Adafruit_MCP4725 _MCP4725;

   public:

    DAC(void);

    void setOutput(uint16_t);
  };

  // ----------------------------------- DAC -----------------------------------




  // =================================== PID ===================================

  class PID_Ctrl {

    double _p, _error, _i, _errorInt, _d, _errorDiff;
    double _dt, _timeLast, _valueLast;
    DAC   *_dacPointer = NULL;

    void   _update(double, double);

   public:

    PID_Ctrl(void);

    PID_Ctrl& attach(DAC* d) { _dacPointer = d;    return *this; };
    PID_Ctrl& detach(void)   { _dacPointer = NULL; return *this; };

    PID_Ctrl& setP  (double p) { _p = p; return *this; }
    PID_Ctrl& setI  (double i) { _i = i; return *this; }
    PID_Ctrl& setD  (double d) { _d = d; return *this; }
    PID_Ctrl& setPID(double p, double i, double d) { _p = p; _i = i; _d = d; return *this; }

    vector<double> getPID(void) const;

    double getOutput(void) const;
    void   regulate (double, double);
    void   autotune (void);
  };

  // ----------------------------------- PID -----------------------------------




  class Heater {

    double _TCR, _res20, _resCable;

   public:

    Sensor_Power sensor;
    DAC          dac;
    PID_Ctrl     pid;

    double   resistance;
    double   power, temperature;
    uint16_t power_set = 10, temperature_set = 200;

    state_t  state = OFF;
    mode_t   mode  = TEMP_MODE;

    Heater(void);

    Heater& setRes20   (double res) { _res20    = res; return *this; }
    Heater& setResCable(double res) { _resCable = res; return *this; }
    Heater& setTCR     (double tcr) { _TCR      = tcr; return *this; }

    Heater& setTemp (uint16_t t) { temperature_set = t; return *this; }
    Heater& setPower(uint16_t p) { power_set       = p; return *this; }

    Heater& setMode (mode_t m) { mode = m; return *this; }

    Heater& on (void) { state = ON;  return *this; }
    Heater& off(void) { state = OFF; return *this; }

    void update   (void);
    void regulate (void);
    void calibrate(void);
  };




  // ================================= CONTROLS ================================

  struct Controls {

    virtual uint8_t read(void) = 0;
  };


  struct Encoder : public Controls {

   private:

    static const uint8_t _LUT[7][4];
    uint8_t _state, _pinCLK, _pinDT;

   public:

    enum dir_t   { DIR_NONE = 0x0, CW = 0x10, CCW = 0x20};
    enum state_t { START, CW_FINAL, CW_BEGIN, CW_NEXT, CCW_BEGIN, CCW_FINAL, CCW_NEXT };

    Encoder(void);

    void    begin(uint8_t, uint8_t);
    uint8_t read(void);
  };


  struct Button : public Controls {

    uint8_t read(void);
  };


  struct Switch : public Controls {

    uint8_t read(void);
  };

  // --------------------------------- CONTROLS --------------------------------




  class Input {

   private:

    static void _isr_encoder(void);
    static void _isr_button (void);

   public:

    enum action_t { NOACTION, DECREASE, INCREASE, PRESS, RELEASE, LONGPRESS };

    static Encoder encoder;
    vector<Button> buttons;
    vector<Switch> switches;

    Input(void);

    void addEncoder(uint8_t, uint8_t);
    void addButton (uint8_t, void *(void));
    void addSwitch (uint8_t, void *(void));

    void update(void);
  };




  class GUI {

   protected:

    Timer timer;

   public:

    Adafruit_SSD1306 display;

    GUI(void);

    void draw (void);
    void clear(void);
  };




  class Settings : public GUI {


  };




  class Infoscreen : public GUI {


  };




  // ================================ VAPORIZER ================================

  extern Heater heater;
  extern Input  input;
  extern GUI    gui;

  void init(uint8_t, uint8_t);

  //void ISR(void);

  static String v_version(void) { return V_FIRMWARE_VERSION; }

  // -------------------------------- VAPORIZER --------------------------------




}


#endif // VAPORIZER_H
