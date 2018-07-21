
#define HEATER_RES20     6.77                    // Resistance of heater at 20°C
#define HEATER_RESCABLE  0.27                    // Resistance of cable

#define I2C_SCL          D5                      // Pin 5
#define I2C_SDA          D4                      // Pin 4
#define ENC_CLK          D3                      // Pin 14
#define ENC_DT           D6                      // Pin 12


//----------------------------------------------------------------------------//

#include <Vaporizer.h>
// #include <MegunoLinkInterface.h>

Vaporizer vape;
uint32_t  counter;


void left () { Serial.println(--counter); }
void right() { Serial.println(++counter); }
void enter() { Serial.println("BANG");    }
void test () { Scheduler::remove("");     }
void counter1() {Serial.println(++counter);}


void setup() {

  Serial.begin(9600);

  vape.begin(I2C_SCL, I2C_SDA);
  // vape.controls.add(Encoder(D4, D5, left, right));
  // vape.controls.add(Button(D6, enter));

  // vape.scheduler.add(Controls::update, 30);
  vape.scheduler.add(counter1, 10);
  vape.scheduler.add(test, 0.5);
  vape.scheduler.run();
}


void loop() {}

//----------------------------------------------------------------------------//
