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
    ( 9.0*resistance + (voltage/constrain(current, 1, 15000) - resCable) )/10.0;

  if (voltage > 100 && current < 10) {                 // if no heater connected
    resistance = res20;
  }

  // power [W]
  power = (float)(power +voltage*current/1000000.0)/2.0f;

  // temperature [°C]
  temperature =
    ( 95.0f*temperature + (float)( TCR_SS316L*log(resistance/res20) + 20.0 )*5.0f )/100.0f;
}

// --------------------------------- SENSOR --------------------------------- //



// =================================== DAC ================================== //

DAC::DAC()
{
  MCP4725.begin(0x62);
  MCP4725.setVoltage(4095, false);             // boot with minimal power output
}

<<<<<<< HEAD
void DAC::setOutput(float x)
{
  if (x < 0)    { x = 0.0f; } else
  if (x > 1.0f) { x = 1.0f; }

  uint16_t val = (uint16_t)(4095.0f*x);

  MCP4725.setVoltage(4095 - val, false);
}

=======
>>>>>>> d37465f52300413dd38d7f61a453d928d4555eda
// ----------------------------------- DAC ---------------------------------- //



// ================================ VAPORIZER =============================== //



// -------------------------------- VAPORIZER ------------------------------- //
