
#define WIRE_FREQ        800000L
#define PWM_RANGE        1023
#define PWM_FREQ         4000                    // Max: CPU_CLOCK / PWM_RANGE

#ifdef ESP8266
  #define I2C_SCL        D1                      // Pin 5
  #define I2C_SDA        D2                      // Pin 4
  #define ENC_CLK        D5                      // Pin 14
  #define ENC_DT         D6                      // Pin 12
  #define ENC_SW         D7                      // Pin 13
  #define BTN_POWER      D8                      // Pin 15
#endif // ESP8266

#ifdef ESP32
  #define I2C_SCL        5                      // Pin 5
  #define I2C_SDA        4                      // Pin 4
#endif // ESP32


//----------------------------------------------------------------------------//


#include "Arduino.h"
#include "PowerSupply.h"


void showArgs();
void showMillis();
void showDelay();
void alert();
void randGen();
void clearScr();


CLI cli;

CmdList cmds = {

  { "args",   "Show arguments for debugging purposes.",     0, &showArgs   },
  { "millis", "Return millis().",                           0, &showMillis },
  { "delay",  "Delay for a specific amount. [ms]",          1, &showDelay  },
  { "alert",  "Trigger alert after time span. [ms]",        1, &alert      },
  { "rand",   "Random number between limits. [low] [high]", 2, &randGen    },
  { "clear",  "Clear contents of screen.",                  0, &clearScr   },
};


//----------------------------------------------------------------------------//


void setup() {

  Serial.begin(115200);
  Serial.println();
  Serial.println();

  cli.begin("> ", &Serial, cmds);

  #ifdef ESP8266
    analogWriteRange(PWM_RANGE);
    analogWriteFreq (PWM_FREQ);
  #endif // ESP8266

  Wire.begin(I2C_SDA, I2C_SCL);              // Select I2C pins
  Wire.setClock(WIRE_FREQ);                  // Faster I2C transmission (800kHz)
}


void loop() {

  while (Serial.available()) {
    cli << Serial.read();
    yield();
  }
}


//----------------------------------------------------------------------------//


void showDelay() {

  uint32_t 
    tstamp = millis(),
    arg    = cli.getArg_i(1);

  while (abs(arg) > (uint32_t)(millis() - tstamp)) {
    delay(100);
    Serial.write('.');
  }
  Serial.println();
  Serial.print("Delayed for ");
  Serial.print(millis() - tstamp);
  Serial.println(" ms.");
}


void showMillis() {

  Serial.println(millis());
}


void showArgs() {

  for(std::string arg : cli.getArgs()) {
    Serial.println(arg.c_str());
  }
}


void clearScr() {

  Serial.print(CHAR_FF);
}


void alert() {
  
  uint32_t arg = cli.getArg_i(1);

  delay(abs(arg));
  Serial.print(CHAR_BEL);
}


void randGen() {

  uint32_t
    low  = cli.getArg_i(1), 
    high = cli.getArg_i(2);

  srand(millis());
  Serial.println(map(rand()%32767, 0, 32766, low, high));
}


//----------------------------------------------------------------------------//
