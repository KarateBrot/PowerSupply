
#define WIRE_FREQ        800000L
#define PWM_RANGE        1023
#define PWM_FREQ         4000                    // Max: CPU_CLOCK / PWM_RANGE

#define I2C_SCL          D1                      // Pin 5
#define I2C_SDA          D2                      // Pin 4
#define ENC_CLK          D5                      // Pin 14
#define ENC_DT           D6                      // Pin 12
#define ENC_SW           D7                      // Pin 13
#define BTN_POWER        D8                      // Pin 15


//----------------------------------------------------------------------------//


#include "Arduino.h"
#include "PowerSupply.h"


void setup() 
{
  Serial.begin(9600);
  Serial.println("");

  analogWriteRange(PWM_RANGE);
  analogWriteFreq (PWM_FREQ);
  
  Wire.begin(I2C_SDA, I2C_SCL);              // Select I2C pins
  Wire.setClock(WIRE_FREQ);                  // Faster I2C transmission (800kHz)
}


void loop() 
{

}


//----------------------------------------------------------------------------//
