
#include <Vaporizer.h>

// #include <MegunoLinkInterface.h>


#define PIN_SCL    5                                                   // Pin D1
#define PIN_SDA    4                                                   // Pin D2
#define PIN_CLK   14
#define PIN_DT    12


Vaporizer vape;


// =================================== RUN =====================================

void setup() {

  Serial.begin(9600);

  vape.begin(SCL, SDA);
}

void loop() {

  vape.heater.update();
  vape.heater.regulate();
  vape.timer.limitCPS(30);
  vape.tick++;
}

// ----------------------------------- RUN -------------------------------------
