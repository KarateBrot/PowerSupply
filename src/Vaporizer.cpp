//############################################################################//
//                                                                            //
//     VAPORIZER.CPP - Library for operating custon built vaporizer based     //
//                     on ESP8266                                             //
// -------------------------------------------------------------------------- //
//                     © 2018, Jan Post                                       //
//                                                                            //
//############################################################################//


#include <Vaporizer.h>




void Service::waitUntil(uint32_t timer) {

  while (micros() - _timer_lastWaitCall < timer) { yield(); }
  _timer_lastWaitCall = micros();
}




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




PID_Ctrl::PID_Ctrl() {

}


// <-- attachInput() here

float PID_Ctrl::getOutput(float value_set, float value) {

  // ------------------------------------------------------------------------ //
  //                         --< PID-CONTROLLER >--                           //
  // -------------------------------------------------------------------------//
  // SIGNAL OUTPUT:   u(t) = P*e(t) + I*∫e(t)dt + D*de(t)/dt                  //
  //                      e(t)     = ΔT                                       //
  //                      ∫e(t)dt  = idle                                     //
  //                      de(t)/dt = -dT/dt                                   //
  // -------------------------------------------------------------------------//
  //              =>  u(t) = P*ΔT + I*idle - D*dT/dt                          //
  // ------------------------------------------------------------------------ //

  // e(t)
  _error = value_set - value;

  // dt
  _dt = (float)(micros() - _timeLast)/1000000.0f;
  _timeLast = micros();

  // de(t)/dt
  _errorDiff = (value - _valueLast)/_dt;
  _valueLast =  value;

  // ∫e(t)dt
  if (_error <= 0.025f) {                                                       // only start integrating shortly before reaching e(t) = 0 (to prevent integral windup)
    _errorInt += _error*_dt + 0.001f;                                           // [+ 0.001]: let P and I fight each other (for "stiffer" temp regulation)
    _errorInt  = constrain(_errorInt, 0, 1);
  }

  // u(t) = P*e(t) + I*∫e(t)dt + D*de(t)/dt
  return constrain(_p*_error + _i*_errorInt - _d*_errorDiff, 0, 1);
}




// ================================= Heater ================================= //

Heater::Heater() {

  pid.set(PID_P, PID_I, PID_D);
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

// Sets DAC pin "OUT" to DC voltage according to PID-controller
void Heater::regulate() {

  float output = pid.getOutput(temperature_set, temperature);
  output *= 4095.0f;
  dac.setOutput((uint16_t)output);
}

// Make sure heater core is at room temperature before calibration!
void Heater::calibrate() {

  sensor.setPrecision(HIGH);
  dac.setOutput(10);

  for (size_t i = 0; i < 30; i++) { fetchData(); }

  dac.setOutput(0);
  setRes20(resistance);
  sensor.setPrecision(LOW);

  pid.set(1.0f, 0.0f, 0.0f);

  // <-- Insert PID-tuning here

  // pid.set(p_ideal, i_ideal, d_ideal);
}

// --------------------------------- Heater --------------------------------- //




// ================================== GUI =================================== //

GUI::GUI() {

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextColor(WHITE);
}

void GUI::clear() {

  display.clearDisplay();
  display.display();
}

void GUI::draw() {

  display.clearDisplay();

  // drawing stuff here

  display.display();
}

// ---------------------------------- GUI ----------------------------------- //




// =============================== VAPORIZER ================================ //



// ------------------------------- VAPORIZER -------------------------------- //
