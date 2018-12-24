#include "Sensor.h"


Adafruit_INA219 Sensor_Load::_INA219;


Sensor_Load::Sensor_Load() {

  _INA219.begin();
}


void Sensor_Load::setPrecisionHigh(bool b) {

  b ? _INA219.setCalibration_16V_400mA() : _INA219.setCalibration_32V_2A();
}


double Sensor_Load::getCurrent() const {

  // Current [mA]
  // R050 instead of R100 shunt resistor (R050: R = 50mΩ | R100: R = 100mΩ)
  // We need to know the actual current flow for a 50mΩ shunt (I050 = U/R050)
  // I050 = U/R050 = U/(0.5*R100) = 2*U/R100 = 2*I100 ─► 2*getCurrent_mA()
  return _INA219.getCurrent_mA()*2.0;
}


double Sensor_Load::getVoltage() const {

  // Voltage [mV]
  return _INA219.getBusVoltage_V()*1000.0;
}