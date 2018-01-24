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
  double voltage, current, resistance, res20, resCable;
  float  power, temperature;

  Sensor(void);

  void read();
};



// =============================== VAPORIZER ================================ //

class Vaporizer
{
public:
  Sensor sensor;
};

// -------------------------------- VAPORIZER ------------------------------- //


#endif // VAPORIZER_H
