
#include <Vaporizer.h>
  using namespace Vaporizer;


#define SCL   5                                                        // Pin D1
#define SDA   4                                                        // Pin D2

Timer timer;

// ================================== RUN =================================== //

void setup() {

  init(SCL, SDA);
}

void loop() {

  heater.update();
  heater.regulate();
  timer.limitFPS(30);
}

// ---------------------------------- RUN ----------------------------------- //
