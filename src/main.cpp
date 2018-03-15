
#include <Vaporizer.h>
  using namespace Vaporizer;

// #include <MegunoLinkInterface.h>


#define PIN_SCL    5                                                   // Pin D1
#define PIN_SDA    4                                                   // Pin D2
#define PIN_CLK   14
#define PIN_DT    12


Timer timer;


// ================================== RUN =================================== //

void setup() {

  Serial.begin(9600);
  init(SCL, SDA);

  // setup_cmd();

  input.addEncoder(PIN_CLK, PIN_DT);
}

void loop() {

  heater.update();
  heater.regulate();
  // sendData();
  // cmd.Process();
  timer.limitCPS(30);
  timer.counter++;
}

// ---------------------------------- RUN ----------------------------------- //
