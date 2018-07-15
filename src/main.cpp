
#define HEATER_RES20     6.77                    // Resistance of heater at 20°C
#define HEATER_RESCABLE  0.27                    // Resistance of cable

#define I2C_SCL          D5                      // Pin 5
#define I2C_SDA          D4                      // Pin 4
#define ENC_CLK          D5                      // Pin 14
#define ENC_DT           D6                      // Pin 12


//----------------------------------------------------------------------------//

#include <Vaporizer.h>
// #include <MegunoLinkInterface.h>

Vaporizer vape;
// bool      state[5];
uint32_t  counter;
//
//
// void sw(bool &b) { b = !b; }
//
// void func1() { sw(state[0]); }
// void func2() { sw(state[1]); }
// void func3() { sw(state[2]); }
// void func4() { sw(state[3]); }
// void func5() { sw(state[4]); }
//
// void draw () {
//
//   vape.gui.display.clearDisplay();
//   vape.gui.display.setTextColor(WHITE);
//   vape.gui.display.setCursor(10, 10);
//
//   for (size_t i = 0; i < 5; i++) {
//     state[i] ? vape.gui.display.print("x") : vape.gui.display.print(" ");
//     vape.gui.display.print("  ");
//   }
//
//   vape.gui.display.setCursor(10, 20);
//   vape.gui.display.print((uint32_t)(++counter/60.0f));
//   vape.gui.display.display();
// }

void left () { Serial.println(--counter); }
void right() { Serial.println(++counter); }
void enter() { Serial.println("BANG");    }


void setup() {

  Serial.begin(9600);

  vape.begin(I2C_SCL, I2C_SDA);
  vape.controls.add(Encoder(D4, D5, left, right));
  vape.controls.add(Button(D6, enter));

  vape.scheduler.add(Ctrl::update, 30);

  // vape.scheduler.add( func1, 1 );
  // vape.scheduler.add( func2, 2 );
  // vape.scheduler.add( func3, 4 );
  // vape.scheduler.add( func4, 8 );
  // vape.scheduler.add( func5, 16 );
  // vape.scheduler.add( draw,  60  );
  // vape.scheduler.add(Scheduler::stop, 1/10.0f);

  vape.scheduler.run();
}


void loop() {}

//----------------------------------------------------------------------------//
