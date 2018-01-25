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



struct Sensor
{
  Adafruit_INA219 INA219;
  double current, voltage, resistance, res20, resCable;
  float  power, temperature;

  Sensor(void);

  void read(void);
};



struct DAC
{
  Adafruit_MCP4725 MCP4725;

  DAC(void);

  void setOutput(float);
};



class Regulation
{
  Sensor sensor;
  DAC    dac;
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
