
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
uint8_t num;

void test1() { Serial.println(num++); }

void setup() {

  Serial.begin(9600);

  vape.controls.add(Button(D6, test1));

  vape.begin(I2C_SCL, I2C_SDA);
}


void loop() {

  vape.run(30);
}

//----------------------------------------------------------------------------//
