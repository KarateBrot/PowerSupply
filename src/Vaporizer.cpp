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
  _INA219.setCalibration_32V_2A();
  setTCR(TCR_SS316L);
}

void Sensor::read()
{
  // Current [mA]
  current = (double)_INA219.getCurrent_mA()*2.0;   // R050 instead of R100 shunt

  // Voltage [mV]
  voltage = (double)_INA219.getBusVoltage_V()*1000.0;

  // Resistance [Ω]
  resistance =
    ( 9.0*resistance + (voltage/constrain(current, 1, 15000) - _resCable) )/10.0;

  if (voltage > 100 && current < 10) {                 // if no heater connected
    resistance = _res20;
  }

  // Power [W]
  power = (float)(power + voltage*current/1000000.0)/2.0f;

  // Temperature [°C]
  temperature =
    ( 95.0f*temperature + (float)( _TCR*log(resistance/_res20) + 20.0 )*5.0f )/100.0f;
}

// -------------------------------- SENSOR ---------------------------------- //



// ================================== DAC =================================== //

DAC::DAC()
{
  _MCP4725.begin(0x62);
  _MCP4725.setVoltage(4095, false);            // boot with minimal power output
}

// Sets DAC pin "OUT" to DC voltage from 0..Vcc
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

// Make sure heater core is at room temperature before calibration!
void Heater::calibrate()
{
  for (size_t i = 0; i < 30; i++) {
    sensor.read();
  }
  sensor.setRes20(sensor.resistance);

  // <-- Insert PID-tuning here

  // setPID(p, i, d);
}

// Sets DAC pin "OUT" to DC voltage according to PID-controller
void Heater::regulate()
{
  // ------------------------------------------------------------------------ //
  //                         --< PID-CONTROLLER >--                           //
  // -------------------------------------------------------------------------//
  // SIGNAL OUTPUT:   u(t) = P*e(t) + I*∫e(t)dt + D*de(t)/dt                  //
  //                      e(t)     = ΔT                                       //
  //                      ∫e(t)dt  = dac_idle                                 //
  //                      de(t)/dt = -dT/dt                                   //
  //                      u(t)     = dac_output                               //
  //            =>    dac_output   = P*ΔT + I*dac_idle - D*dT/dt              //
  // ------------------------------------------------------------------------ //

  // ΔT [°C]
  _dTemp = (float)temperature_set - sensor.temperature;

  // Time step for I and D
  float _dt = (micros() - _timeLast)/1000000.0;
  _timeLast = micros();

  // dT/dt [°C/s]
  _dTdt     = (sensor.temperature - _tempLast)/_dt;
  _tempLast = sensor.temperature;

  // ∫ΔTdt [°C*s]
  if (_dTemp <= 5.0) {                                                          // only start integrating shortly before reaching ΔT = 0 (to prevent T from overshooting)
    _idle += _dTemp*_dt + 0.05;                                                 // [+ 1.0]: let P and I fight each other (for "stiffer" temp regulation)
    _idle  = constrain(_idle, 0, 4095);
  }

  _output = (uint16_t)constrain(_p*_dTemp + _i*_idle - _d*_dTdt, 0, 4095);

  dac.setOutput(_output);
}

// --------------------------------- Heater --------------------------------- //



// ================================ VAPORIZER =============================== //



// -------------------------------- VAPORIZER ------------------------------- //
