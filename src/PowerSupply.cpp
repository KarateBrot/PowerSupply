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
void Tools::smoothExp(double &x, double val, float weight) {

  x = (1-weight)*x + weight*val;
}

// Advanced exponential smoothing (simulates capacitor)
void Tools::smoothExp(double &x, double val, uint32_t sampleTime, uint32_t timeConst) {
  
  double weight = 1 - exp(-1.0*sampleTime/timeConst);
  x = (1-weight)*x + weight*val;
}

// Hysteresis (simulates Inverting Schmitt Trigger)
bool Tools::trigger(bool &trigger, double val, float lim_lower, float lim_upper) {
  
  bool trigger_last = trigger;

  if (val >= lim_upper) { trigger = true;  } else
  if (val <  lim_lower) { trigger = false; }

  if (trigger_last != trigger) { return true; } else { return false; }
}

// Set ESP8266 pins HIGH or LOW very fast
void Tools::digitalWriteFast(uint8_t pin, bool val) {

  // Register for setting pin HIGH: PERIPHS_GPIO_BASEADDR + 4
  // Register for setting pin LOW:  PERIPHS_GPIO_BASEADDR + 8
  WRITE_PERI_REG(PERIPHS_GPIO_BASEADDR + 4*(val + 1), 1 << pin);
}

// Constrains (and modifies) a value according to lower and upper limit
template <typename T, typename U>
void Tools::trim(T &val, U lim_lower, U lim_upper) {

  if (val < lim_lower) { val = lim_lower; } else
  if (val > lim_upper) { val = lim_upper; }
}

// ----------------------------------- TOOLS -----------------------------------




// ================================ POWERSUPPLY ================================

Sensor_Load PowerSupply::sensor;
PID         PowerSupply::pid;
DAC         PowerSupply::dac;

double PowerSupply::voltage;
double PowerSupply::current;
double PowerSupply::power;

PowerSupply::PowerSupply() {

  _voltage_set = 0;
  _current_set = 0;
  _power_set   = 0;
  _mode        = VOLTAGE_MODE;
  
  pid.setPID(PID_P, PID_I, PID_D);
}

void PowerSupply::converge(double &value, double value_set) {

  pid.update(value, value_set);
  uint16_t output = (uint16_t)( pid.getOutput()*4095.0 + 0.5 );
  dac.setOutput(output);
}

void PowerSupply::increment(int16_t val) {

  switch (_mode) {

    case VOLTAGE_MODE:
      _voltage_set += val;
      Tools::trim(_voltage_set, VOLTAGE_MIN, VOLTAGE_MAX);
      break;

    case CURRENT_MODE:
      _current_set += val;
      Tools::trim(_current_set, CURRENT_MIN, CURRENT_MAX);

    case POWER_MODE:
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

      case VOLTAGE_MODE:
        converge(voltage, _voltage_set);
        break;
    
      case CURRENT_MODE:
        converge(current, _current_set);
        break;

      case POWER_MODE:
        converge(power, _power_set);
        break;

      default:
        converge(voltage, _voltage_set);
        break;
    }
    sensor.setPrecision(LOW);

  } else {

    dac.setOutput(0);
    sensor.setPrecision(HIGH);
  }
}

// -------------------------------- POWERSUPPLY --------------------------------




// =================================== Heater ==================================

double Heater::resistance;
double Heater::temperature;

Heater::Heater() {

  setTCR     (HEATER_TCR);
  setRes20   (HEATER_RES20);
  setResCable(HEATER_RESCABLE);

  _mode       = TEMP_MODE;
  resistance  = HEATER_RES20;
  temperature = 20.0;
}

// Make sure heater core is at room temperature before calibration!
void Heater::calibrate() {

  pid.setPID(1.0, 0.0, 0.0);                  // TODO: Manual tuning
  sensor.setPrecision(HIGH);

  // Make sure current flow is 10mA +/- 1mA
  double currentLast = 0.0;
  while (abs(current - 10.0) > 1.0 && abs(currentLast - 10.0) > 1.0) {
    currentLast = current;
    update();
    PowerSupply::converge(current, 10.0);
    yield();
  }

  // Calculate resistance using reference current of 10mA
  for (size_t i = 0; i < 15; i++) {
    update();
    PowerSupply::converge(current, 10.0);
    yield();
  }

  // TODO: Room temp measurement to compensate res for temps != 20°C

  dac.setOutput(0);
  setRes20(resistance);
  sensor.setPrecision(LOW);
}

void Heater::increment(int16_t val) {

  if (_mode == TEMP_MODE) {

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
    0.3*(voltage/constrain(current, 1, 15000) - _resCable);

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
    _mode == TEMP_MODE
      ? PowerSupply::converge(temperature, _temperature_set)
      : PowerSupply::converge(power, _power_set);
    sensor.setPrecision(LOW);

  } else {

    dac.setOutput(0);
    sensor.setPrecision(HIGH);
  }
}

// ----------------------------------- Heater ----------------------------------