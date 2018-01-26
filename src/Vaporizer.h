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



class Sensor
{
  Adafruit_INA219 _INA219;
  double          _res20, _resCable;

 public:

  double current, voltage, resistance;
  float  power, temperature;

  Sensor(void);

  void read        (void);
  void set_res20   (double);
  void set_resCable(double);
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
  float _p, _i, _d;

 public:

  Sensor sensor;
  DAC    dac;

  Heater(float, float, float);

  void set_PID  (float, float, float);
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

};

// -------------------------------- VAPORIZER ------------------------------- //


#endif // VAPORIZER_H
