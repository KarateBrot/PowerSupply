
// #define HEATER_RES20     6.77                 // Resistance of heater at 20°C
// #define HEATER_RESCABLE  0.27                 // Resistance of cable

#define PIN_SCL          5                    // Pin D1
#define PIN_SDA          4                    // Pin D2
#define PIN_CLK         14
#define PIN_DT          12


//----------------------------------------------------------------------------//

#include <Vaporizer.h>
// #include <MegunoLinkInterface.h>


Vaporizer vape;


void setup() {

  Serial.begin(9600);
  vape.begin(PIN_SCL, PIN_SDA);
}


void loop() {

  vape.heater.update();
  vape.heater.regulate();
  vape.timer.limitCPS(30);
  vape.tick++;
}

//----------------------------------------------------------------------------//
