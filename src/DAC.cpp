#include "DAC.h"


Adafruit_MCP4725 DAC::_MCP4725;


DAC::DAC() {

  _MCP4725.begin(0x62);
  _MCP4725.setVoltage(4095, false);            // boot with minimal power output
}


// Applies DC voltage from 0..Vcc at pin "OUT" on MCP4725 breakout board
void DAC::setOutput(uint16_t val) {

  _MCP4725.setVoltage(4095 - val, false);
}