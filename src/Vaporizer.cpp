//############################################################################//
//                                                                            //
//     VAPORIZER.CPP - Library for operating custon built vaporizer based     //
//                     on ESP8266                                             //
// -------------------------------------------------------------------------- //
//                     © 2018, Jan Post                                       //
//                                                                            //
//############################################################################//


#include <Vaporizer.h>



// ================================= SENSOR ================================= //

Sensor::Sensor()
{
  INA219.begin();
}

void Sensor::read()
{
  // current [mA]
  current = (double)INA219.getCurrent_mA()*2.0;

  // voltage [mV]
  voltage = (double)INA219.getBusVoltage_V()*1000.0;

  // resistance [Ω]
  resistance =
    ( 9.0*resistance +(voltage/constrain(current, 1, 15000) -resCable) ) /10.0;
  if (voltage > 100 && current < 10){ resistance = res20; }                     // if no heater connected

  // power [W]
  power = (float)(power +voltage*current/1000000.0)/2.0f;

  // temperature [°C]
  temperature =
    ( 95.0f*temperature + (float)( TCR_SS316L*log(resistance/res20) + 20.0 )*5.0f )/100;
}

// --------------------------------- SENSOR --------------------------------- //



// ================================ VAPORIZER =============================== //



// -------------------------------- VAPORIZER ------------------------------- //
