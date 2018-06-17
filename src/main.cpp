
// #define HEATER_RES20     6.77                 // Resistance of heater at 20°C
// #define HEATER_RESCABLE  0.27                 // Resistance of cable

#define I2C_SCL          5                    // Pin D1
#define I2C_SDA          4                    // Pin D2
#define ENC_CLK         14
#define ENC_DT          12


//----------------------------------------------------------------------------//

#include <Vaporizer.h>
// #include <MegunoLinkInterface.h>


Vaporizer vape;


void setup() {

  Serial.begin(9600);
  vape.begin(I2C_SCL, I2C_SDA);
}


void loop() {

  vape.run(30);
}

//----------------------------------------------------------------------------//
