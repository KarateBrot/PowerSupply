
#define HEATER_RES20     6.77                    // Resistance of heater at 20°C
#define HEATER_RESCABLE  0.27                    // Resistance of cable

#define I2C_SCL          D1                      // Pin 5
#define I2C_SDA          D2                      // Pin 4
#define ENC_CLK          D5                      // Pin 14
#define ENC_DT           D6                      // Pin 12


//----------------------------------------------------------------------------//

#include <Vaporizer.h>
// #include <MegunoLinkInterface.h>


Vaporizer vape;
uint32_t  test1, test2;

void loop1() { test1++; Serial.print(test1); Serial.print("-"); Serial.println(test2); }
void loop2() { test2++; Serial.print(test1); Serial.print(" "); Serial.println(test2); }
void loop3() { Serial.println("-----"); }


void setup() {

  Serial.begin(9600);

  vape.begin(I2C_SCL, I2C_SDA);

  vape.timer.add(loop1,       0.3  );
  vape.timer.add(loop2,       0.5  );
  vape.timer.add(loop3,       0.1  );
  vape.timer.add(Timer::stop, 0.025);

  vape.timer.run();
}


void loop() {


}

//----------------------------------------------------------------------------//
