
#define HEATER_RES20     6.77                    // Resistance of heater at 20°C
#define HEATER_RESCABLE  0.27                    // Resistance of cable

#define I2C_SDA          D4                      // Pin 4
#define I2C_SCL          D5                      // Pin 5
#define ENC_CLK          D3                      // Pin 14
#define ENC_DT           D6                      // Pin 12

#define WIRE_FREQ        800000L
#define PWM_RANGE        1023
#define PWM_FREQ         4000                    // Max: CPU_CLOCK / PWM_RANGE


//----------------------------------------------------------------------------//


#include "Vaporizer.h"


Scheduler timer;
Heater    heater;
Controls  controls;


void loop1() {

  heater.update();
  heater.regulate();
  controls.update();
}


void setup() {

  Serial.begin(9600);

  Wire.begin(I2C_SDA, I2C_SCL);              // Select I2C pins
  Wire.setClock(WIRE_FREQ);                  // Faster I2C transmission (800kHz)
 
  analogWriteRange(PWM_RANGE);
  analogWriteFreq (PWM_FREQ);

  timer.add(loop1).frequency(30);
  timer.run();
}


void loop() {}                               // Does not get called


//----------------------------------------------------------------------------//
