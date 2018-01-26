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
  _INA219.begin();
}

void Sensor::read()
{
  // current [mA]
  current = (double)_INA219.getCurrent_mA()*2.0;   // R050 instead of R100 shunt

  // voltage [mV]
  voltage = (double)_INA219.getBusVoltage_V()*1000.0;

  // resistance [Ω]
  resistance =
    ( 9.0*resistance + (voltage/constrain(current, 1, 15000) - _resCable) )/10.0;

  if (voltage > 100 && current < 10) {                 // if no heater connected
    resistance = _res20;
  }

  // power [W]
  power = (float)(power + voltage*current/1000000.0)/2.0f;

  // temperature [°C]
  temperature =
    ( 95.0f*temperature + (float)( TCR_SS316L*log(resistance/_res20) + 20.0 )*5.0f )/100.0f;
}

// -------------------------------- SENSOR ---------------------------------- //



// ================================== DAC =================================== //

DAC::DAC()
{
  _MCP4725.begin(0x62);
  _MCP4725.setVoltage(4095, false);            // boot with minimal power output
}

void DAC::setOutput(uint16_t val)
{
  _MCP4725.setVoltage(4095 - val, false);
}

// ---------------------------------- DAC ----------------------------------- //



// ================================= Heater ================================= //

Heater::Heater(float p, float i, float d)
{
  _p = p;
  _i = i;
  _d = d;
}

void Heater::set_PID(float p, float i, float d)
{
  _p = p;
  _i = i;
  _d = d;
}

// Make sure heater core is at room temperature before calibration!
void Heater::calibrate()
{
  for (size_t i = 0; i < 30; i++) {
    sensor.read();
  }
  sensor.set_res20(sensor.resistance);

  // <-- Insert PID-tuning here

  // set_PID(p, i, d);
}

void Heater::regulate()
{
  
}

// --------------------------------- Heater --------------------------------- //



// ================================ VAPORIZER =============================== //



// -------------------------------- VAPORIZER ------------------------------- //
