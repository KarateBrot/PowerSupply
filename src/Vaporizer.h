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
  #include <img/splash.h>                                                     //
// -------------------------------------------------------------------------- //
  #define TCR_SS316L 0.00092      // at 20°C                                  //
// -------------------------------------------------------------------------- //
  #define PID_P      850.0f                                                   //
  #define PID_I      100.0f                                                   //
  #define PID_D       80.0f                                                   //
// -------------------------------------------------------------------------- //



class Service
{
  uint32_t _time = 0;

public:

  void     startRuntime(void)       { _time = micros(); }
  uint32_t getRuntime  (void) const { return micros() - _time; };
};



class Sensor
{
  Adafruit_INA219 _INA219;
  double          _TCR, _res20, _resCable;

 public:

  double current, voltage, resistance;
  float  power, temperature;

  Sensor(void);

  void read       (void);
  void setRes20   (double res) { _res20    = res; }
  void setResCable(double res) { _resCable = res; }
  void setTCR     (double tcr) { _TCR      = tcr; }
};



class DAC
{
  Adafruit_MCP4725 _MCP4725;

 public:

  DAC(void);

  void setOutput(uint16_t);
};



class Heater
{
  float    _p, _i, _d;
  float    _dTemp, _dt, _timeLast, _dTdt, _tempLast, _idle;
  uint16_t _output;

 public:

  Sensor   sensor;
  DAC      dac;
  uint16_t temperature_set;

  Heater(float, float, float);

  void setPID   (float p, float i, float d) { _p = p; _i = i; _d = d; }
  void setPID_P (float p)                   { _p = p; }
  void setPID_I (float i)                   { _i = i; }
  void setPID_D (float d)                   { _d = d; }
  void calibrate(void);
  void regulate (void);
};



class Input
{

};



class GUI
{

};



// =============================== VAPORIZER ================================ //

class Vaporizer
{
 public:

   Heater heater;
};

// -------------------------------- VAPORIZER ------------------------------- //


#endif // VAPORIZER_H
