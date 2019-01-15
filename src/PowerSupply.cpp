//############################################################################//
//                                                                            //
//     POWERSUPPLY.CPP - Library for operating custon built power supply      //
//                       based on the ESP8266                                 //
// -------------------------------------------------------------------------- //
//                     © 2018, Jan Post                                       //
//                                                                            //
//############################################################################//


#include "PowerSupply.h"




// =================================== TOOLS ===================================

// Simple exponential smoothing
void Tools::smoothExp(double &x, const double &val, const float &weight) {

  x = (1.0-weight)*x + weight*val;
}

// Advanced exponential smoothing (simulates capacitor)
void Tools::smoothExp(double &x, const double &val, const uint32_t &sampleTime, const uint32_t &timeConst) {
  
  double weight = 1 - exp(-sampleTime/(double)timeConst);
  x = (1-weight)*x + weight*val;
}

// Hysteresis (simulates Inverting Schmitt Trigger)
bool Tools::trigger(bool &trigger, const double &val, const float &low, const float &high) {
  
  bool trigger_last = trigger;

  if (val > high) { trigger = true;  } else
  if (val < low ) { trigger = false; }

  if (trigger_last != trigger) { return true; } else { return false; }
}

// Constrains (and modifies) a value according to lower and upper limit
template <typename T, typename U>
T Tools::limit(const T &val, const U &low, const U &high) {

  if (val > high) { return (T)high; } else
  if (val < low ) { return (T)low;  } else
  return val;
}

// Constrains (and modifies) a value according to lower and upper limit
template <typename T, typename U>
bool Tools::trim(T &val, const U &low, const U &high) {

  if (val > high) { val = high; return true; } else
  if (val < low ) { val = low;  return true; } else
  return false;
}

// ----------------------------------- TOOLS -----------------------------------




// ================================ POWERSUPPLY ================================

Sensor_Load PowerSupply::sensor;
DAC         PowerSupply::dac;

double PowerSupply::voltage;
double PowerSupply::current;
double PowerSupply::power;

PowerSupply::PowerSupply() : 

  _mode(Mode::VOLTAGE) {

  pid.setPID(PID_P, PID_I, PID_D);
}

void PowerSupply::_converge(const double &val, const double &val_set) {

  pid.update(val, val_set, micros());
  uint16_t output = (uint16_t)(pid.getOutput()*4095.0 + 0.5);
  dac.setOutput(output);
}

void PowerSupply::increment(const int16_t &val) {

  switch (_mode) {

    case Mode::VOLTAGE:
      _voltage_set += val;
      Tools::trim(_voltage_set, VOLTAGE_MIN, VOLTAGE_MAX);
      break;

    case Mode::CURRENT:
      _current_set += val;
      Tools::trim(_current_set, CURRENT_MIN, CURRENT_MAX);

    case Mode::POWER:
      _power_set += val;
      Tools::trim(_power_set, POWER_MIN, POWER_MAX);
      break;

    default:
      break;
  }
}

void PowerSupply::update() {

  // Voltage [mV]
  voltage =
    0.5*voltage +
    0.5*sensor.getVoltage();

  // Current [mA]
  current =
    0.5*current +
    0.5*sensor.getCurrent();

  // Power [W]
  power =
    0.5*power +
    0.5*voltage*current/1000000.0;
}

void PowerSupply::regulate() {

  if (_running) {

    switch (_mode) {

      case Mode::VOLTAGE:
        _converge(voltage, _voltage_set);
        break;
    
      case Mode::CURRENT:
        _converge(current, _current_set);
        break;

      case Mode::POWER:
        _converge(power, _power_set);
        break;

      default:
        _converge(voltage, _voltage_set);
        break;
    }
    sensor.setPrecisionHigh(false);

  } else {

    dac.setOutput(0);
    sensor.setPrecisionHigh(true);
  }
}

// -------------------------------- POWERSUPPLY --------------------------------




// =================================== Heater ==================================

double Heater::resistance;
double Heater::temperature;

Heater::Heater() :

  _TCR     (HEATER_TCR), 
  _res20   (HEATER_RES20), 
  _resCable(HEATER_RESCABLE) {

  _mode       = Mode::TEMPERATURE;
  resistance  = HEATER_RES20;
  temperature = 20.0;
}

// Make sure heater core is at room temperature before calibration!
void Heater::calibrate() {

  pid.setPID(1.0, 0.0, 0.0);                  // TODO: Manual tuning
  sensor.setPrecisionHigh(true);

  // Make sure current flow is 10mA +/- 1mA
  double currentLast = 0.0;
  while (std::abs(current - 10.0) > 1.0 && std::abs(currentLast - 10.0) > 1.0) {
    currentLast = current;
    update();
    _converge(current, 10.0);
    
    #ifdef ESP8266
      yield();
    #endif // ESP8266
  }

  // Calculate resistance using reference current of 10mA
  for (size_t i = 0; i < 15; i++) {
    update();
    _converge(current, 10.0);

    #ifdef ESP8266
      yield();
    #endif // ESP8266
  }

  // TODO: Room temp measurement to compensate res for temps != 20°C

  dac.setOutput(0);
  setRes20(resistance);
  sensor.setPrecisionHigh(false);
}

void Heater::increment(const int16_t &val) {

  if (_mode == Mode::TEMPERATURE) {

    _temperature_set += val;
    Tools::trim(_temperature_set, TEMPERATURE_MIN, TEMPERATURE_MAX);

  } else {

    PowerSupply::increment(val);
  }
}

void Heater::update() {

  PowerSupply::update();

  // Resistance [Ω]
  resistance =
    0.7*resistance +
    0.3*(voltage/Tools::limit(current, 1, 15000) - _resCable);

  // Resistance [Ω] - if no heater connected
  if (voltage > 100 && current < 10) { resistance = _res20; }

  // Temperature [°C]
  temperature =
    0.7*temperature +
    0.3*(log(resistance/_res20)/_TCR + 20.0);
}

// Sets DAC pin "OUT" to DC voltage according to PID-controller
void Heater::regulate() {

  if (_running) {

    // TODO: Set ideal PID consts for each mode
    _mode == Mode::TEMPERATURE
      ? _converge(temperature, _temperature_set)
      : _converge(power, _power_set);
    sensor.setPrecisionHigh(false);

  } else {

    dac.setOutput(0);
    sensor.setPrecisionHigh(true);
  }
}

// ----------------------------------- Heater ----------------------------------