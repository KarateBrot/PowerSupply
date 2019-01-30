//############################################################################//
//                                                                            //
//     POWERSUPPLY.CPP - Library for operating custon built power supply      //
//                       based on the ESP8266                                 //
// -------------------------------------------------------------------------- //
//                     © 2018, Jan Post                                       //
//                                                                            //
//############################################################################//


#include "PowerSupply.h"


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
      tools::trim(_voltage_set, VOLTAGE_MIN, VOLTAGE_MAX);
      break;

    case Mode::CURRENT:
      _current_set += val;
      tools::trim(_current_set, CURRENT_MIN, CURRENT_MAX);

    case Mode::POWER:
      _power_set += val;
      tools::trim(_power_set, POWER_MIN, POWER_MAX);
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
  }

  // Calculate resistance using reference current of 10mA
  for (size_t i = 0; i < 15; i++) {
    update();
    _converge(current, 10.0);
  }

  // TODO: Room temp measurement to compensate res for temps != 20°C

  dac.setOutput(0);
  setRes20(resistance);
  sensor.setPrecisionHigh(false);
}

void Heater::increment(const int16_t &val) {

  if (_mode == Mode::TEMPERATURE) {

    _temperature_set += val;
    tools::trim(_temperature_set, TEMPERATURE_MIN, TEMPERATURE_MAX);

  } else {

    PowerSupply::increment(val);
  }
}

void Heater::update() {

  PowerSupply::update();

  // Resistance [Ω]
  resistance =
    0.7*resistance +
    0.3*(voltage/tools::limit(current, 1, 15000) - _resCable);

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