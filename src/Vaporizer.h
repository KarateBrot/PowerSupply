//############################################################################//
//                                                                            //
//     VAPORIZER.H - Library for operating custon built vaporizer based       //
//                   on ESP8266                                               //
// -------------------------------------------------------------------------- //
//                   © 2018, Jan Post                                         //
//                                                                            //
//############################################################################//


#ifndef VAPORIZER_H
#define VAPORIZER_H


// -------------------------------------------------------------------------- //
  #include <Arduino.h>                                                        //
  #include <Adafruit_INA219.h>                                                //
  #include <Adafruit_MCP4725.h>                                               //
  #include <Adafruit_GFX.h>                                                   //
  #include <Adafruit_SSD1306.h>                                               //
// -------------------------------------------------------------------------- //
  #include <vector>                                                           //
    using namespace std;                                                      //
// -------------------------------------------------------------------------- //
  #include <img/splash.h>                                                     //
// -------------------------------------------------------------------------- //
  #define V_FIRMWARE_VERSION     "0.1-a"                                      //
// -------------------------------------------------------------------------- //
  #define HEATER_TCR_SS316L       0.00092                         // at 20°C  //
  #define HEATER_TCR_SS316        0.000915                                    //
  #define HEATER_TCR_SS317L       0.00088                                     //
  #define HEATER_TCR_SS317        0.000875                                    //
  #define HEATER_TCR_SS304        0.00105                                     //
  #define HEATER_TCR_SS410        0.00155                                     //
// -------------------------------------------------------------------------- //
  #define HEATER_RES20            0.51                                        //
  #define HEATER_RESCABLE         0.27                                        //
// -------------------------------------------------------------------------- //
  #define PID_P                   1.0                                         //
  #define PID_I                   0.1                                         //
  #define PID_D                   0.08                                        //
// -------------------------------------------------------------------------- //




namespace Vaporizer {




  class Timer {

    uint32_t _time, _lastWaitCall, _lastCycle;

   public:

    Timer(void);

    void     resetTime(void)       { _time = micros(); }
    uint32_t getTime  (void) const { return micros() - _time; }
    void     waitUntil(uint32_t);
    void     limitFPS (uint8_t);
    float    getFPS   (void);
  };




  class Sensor {

    static Adafruit_INA219 _INA219;

   public:

    double current, voltage;

    Sensor(void);

    void setPrecision(bool);
    void read        (void);
  };




  class DAC {

    static Adafruit_MCP4725 _MCP4725;

   public:

    DAC(void);

    void setOutput(uint16_t);
  };




  class PID_Ctrl {

    double _p, _error, _i, _errorInt, _d, _errorDiff;
    double _dt, _timeLast, _valueLast;

   public:

    PID_Ctrl(void);

    PID_Ctrl& setP  (double p) { _p = p; return *this; }
    PID_Ctrl& setI  (double i) { _i = i; return *this; }
    PID_Ctrl& setD  (double d) { _d = d; return *this; }
    PID_Ctrl& setPID(double p, double i, double d) { _p = p; _i = i; _d = d; return *this; }

    double getOutput     (double, double);
    void   setOutputOfDAC(DAC&, double, double);
    void   autotune      (void);
  };




  class Heater {

    double _TCR, _res20, _resCable;

   public:

    Sensor   sensor;
    DAC      dac;
    PID_Ctrl pid;

    double   resistance;
    double   power, temperature;
    uint16_t power_set = 0, temperature_set = 200;

    Heater(void);

    Heater& setRes20   (double res) { _res20    = res; return *this; }
    Heater& setResCable(double res) { _resCable = res; return *this; }
    Heater& setTCR     (double tcr) { _TCR      = tcr; return *this; }

    void update   (void);
    void regulate (void);
    void calibrate(void);
  };




  class Input {


  };




  class GUI {

   protected:

    static uint32_t    frameCount;
    static vector<GUI> windowBuffer;

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




  // ============================== VAPORIZER =============================== //

  extern Timer  timer;
  extern Heater heater;
  extern Input  input;
  extern GUI    gui;

  void init(uint8_t, uint8_t);

  static String getVersion(void) { return V_FIRMWARE_VERSION; }


  // ------------------------------- VAPORIZER ------------------------------ //




}



#endif // VAPORIZER_H
