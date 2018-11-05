#ifndef DAC_H
#define DAC_H

#include <Adafruit_MCP4725.h>                     // Digital to analog converter


struct DAC {

private:
  static Adafruit_MCP4725 _MCP4725;

public:
  DAC(void);

  void setOutput(uint16_t);
};


#endif // DAC_H