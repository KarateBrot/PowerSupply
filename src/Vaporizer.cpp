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

Sensor::Sensor() {

  _INA219.begin();
  _INA219.setCalibration_32V_2A();
}

void Sensor::setPrecision(bool b) {

  b ? _INA219.setCalibration_16V_400mA() : _INA219.setCalibration_32V_2A();
}

void Sensor::read() {

  // Current [mA]
  current = (double)_INA219.getCurrent_mA()*2.0;   // R050 instead of R100 shunt

  // Voltage [mV]
  voltage = (double)_INA219.getBusVoltage_V()*1000.0;
}

// -------------------------------- SENSOR ---------------------------------- //




// ================================== DAC =================================== //

DAC::DAC() {

  _MCP4725.begin(0x62);
  _MCP4725.setVoltage(4095, false);            // boot with minimal power output
}

// Applies DC voltage from 0..Vcc at pin "OUT" on MCP4725 breakout board
void DAC::setOutput(uint16_t val) {

  _MCP4725.setVoltage(4095 - val, false);
}

// ---------------------------------- DAC ----------------------------------- //




// ================================= Heater ================================= //

Heater::Heater() {

  _p = PID_P;
  _i = PID_I;
  _d = PID_D;
  setTCR(TCR_SS316L);
}

void Heater::fetchData() {

  sensor.read();

  // Resistance [Ω]
  resistance =
    ( 9.0*resistance + (sensor.voltage/constrain(sensor.current, 1, 15000) - _resCable) )/10.0;

  if (sensor.voltage > 100 && sensor.current < 10) {   // if no heater connected
    resistance = _res20;
  }

  // Power [W]
  power = (float)(power + sensor.voltage*sensor.current/1000000.0)/2.0f;

  // Temperature [°C]
  temperature =
    ( 95.0f*temperature + (float)( _TCR*log(resistance/_res20) + 20.0 )*5.0f )/100.0f;
}

// Make sure heater core is at room temperature before calibration!
void Heater::calibrate() {

  sensor.setPrecision(HIGH);
  dac.setOutput(10);

  for (size_t i = 0; i < 30; i++) { fetchData(); }

  dac.setOutput(0);
  setRes20(resistance);
  sensor.setPrecision(LOW);

  // <-- Insert PID-tuning here
  

  // setPID(p, i, d);
}

// Sets DAC pin "OUT" to DC voltage according to PID-controller
void Heater::regulate() {

  // ------------------------------------------------------------------------ //
  //                         --< PID-CONTROLLER >--                           //
  // -------------------------------------------------------------------------//
  // SIGNAL OUTPUT:   u(t) = P*e(t) + I*∫e(t)dt + D*de(t)/dt                  //
  //                      e(t)     = ΔT                                       //
  //                      ∫e(t)dt  = idle                                     //
  //                      de(t)/dt = -dT/dt                                   //
  //                      u(t)     = output                                   //
  // -------------------------------------------------------------------------//
  //                  =>  output = P*ΔT + I*idle - D*dT/dt                    //
  // ------------------------------------------------------------------------ //

  // ΔT [°C]
  _dTemp = (float)temperature_set - temperature;

  // Time step for I and D
  _dt = (micros() - _timeLast)/1000000.0;
  _timeLast = micros();

  // dT/dt [°C/s]
  _dTdt = (temperature - _temperatureLast)/_dt;
  _temperatureLast = temperature;

  // ∫ΔTdt [°C*s]
  if (_dTemp <= 5.0) {                                                          // only start integrating shortly before reaching ΔT = 0 (to prevent integral windup)
    _idle += _dTemp*_dt + 0.05;                                                 // [+ 1.0]: let P and I fight each other (for "stiffer" temp regulation)
    _idle  = constrain(_idle, 0, 4095);
  }

  _output = (uint16_t)constrain(_p*_dTemp + _i*_idle - _d*_dTdt, 0, 4095);

  dac.setOutput(_output);
}

// --------------------------------- Heater --------------------------------- //




// ================================== GUI =================================== //

GUI::GUI() {

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextColor(WHITE);
}

// ---------------------------------- GUI ----------------------------------- //




// =============================== VAPORIZER ================================ //



// ------------------------------- VAPORIZER -------------------------------- //
