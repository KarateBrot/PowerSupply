//############################################################################//
//                                                                            //
//     VAPORIZER.CPP - Library for operating custon built vaporizer based     //
//                     on ESP8266                                             //
// -------------------------------------------------------------------------- //
//                     © 2017, Jan Post                                       //
//                                                                            //
//############################################################################//


#include <Vaporizer.h>



// ================================ VAPORIZER =============================== //

// Needs to get called once at startup ("global constructor")
void Vaporizer::init()
{
  Input::init();
  //trololol
  GUI::init();
  analogWriteRange(PWM_RANGE);
  analogWriteFreq(PWM_FREQ);
  Wire.setClock(800000L);
}

// Needs to be looped to monitor important system functions
// Don't mess with it!
void Vaporizer::monitor()
{
  Input::monitor();
  limitFPS(FPS_MAX);
  getFPS();
  cycleCount++;

}

// Idle until time span since last call is reached
void Vaporizer::waitUntil(unsigned long timer, magnitude_t order)
{
  timer *= pow(10, order - MICRO);
  while (micros() - _time_lastWaitCall < timer) { yield(); }
  _time_lastWaitCall = micros();
}

// Desired framerate for system, needs to be looped to work
void Vaporizer::limitFPS(uint8_t fps)
{
  waitUntil( (unsigned long)(1000000.0f/fps + 0.5f), MICRO );
}

void Vaporizer::getFPS()
{
  _time_lastCycle = micros() - _time_lastCycle;
  framerate = 1000000.0f/_time_lastCycle;
  _time_lastCycle = micros();
}

// Exponential smoothing (simulates capacitor)
void Vaporizer::smooth(double &x, double val, unsigned long sampleTime, unsigned long timeConst)
{
  double weight = 1 - exp(-1.0*sampleTime/timeConst);
  x = (1-weight)*x + weight*val;
}

// Hysteresis (simulates Inverting Schmitt Trigger)
void Vaporizer::trigger(bool &trigger, double val, float lim_lower, float lim_upper)
{
  if (val >= lim_upper) { trigger = true;  } else
  if (val <  lim_lower) { trigger = false; };
}



// ================================== INPUT ================================= //

void Input::init()
{
  pinMode(INPUT_PIN_CLK,   INPUT);
  pinMode(INPUT_PIN_DT,    INPUT);
  pinMode(INPUT_PIN_SW,    INPUT);
  pinMode(INPUT_PIN_POWER, INPUT);

  attachInterrupt(INPUT_PIN_CLK, ISR, CHANGE);
  attachInterrupt(INPUT_PIN_DT,  ISR, CHANGE);
  attachInterrupt(INPUT_PIN_SW,  ISR, CHANGE);

  Encoder::init();
}

void Input::monitor()
{
  Encoder::monitor();
  PowerSwitch::monitor();
}

// Needs Input::monitor() to be looped to function properly
void Input::ISR()
{
  Encoder::ISR();
}

void Input::execute(cmd_t command)
{
  switch (command) {

    case LEFT:
      Item::buffer[0].changeValue(1);
      execute(RIGHT);
      break;

    case RIGHT:
      break;

    case PRESS:
      break;

    case RELEASE:
      break;

    case SELECT:
      break;

    case LONGSELECT:
      break;

  }
}


// ---------------------------- INPUT / ENCODER ----------------------------- //

void Encoder::init()
{
  for (size_t i = 0; i < 3; i++) { read(); }
}

void Encoder::monitor()
{
  ISR();
}

void Encoder::ISR()
{
  int8_t state_last = _state;
  read();
  // In case _state changed, trigger execution of encoder command
  if (_state != state_last) { cmd_t cmd = _getCommand(_state); execute(cmd); }
}

void Encoder::read()
{
  // Order in array: { DT = 0, CLK = 1, SW = 2 }
  _state_raw[SW]  = !digitalRead(INPUT_PIN_SW);
  _state_raw[CLK] =  digitalRead(INPUT_PIN_CLK);
  _state_raw[DT]  =  digitalRead(INPUT_PIN_DT);

  _debounce();
  _refreshState();
}

void Encoder::_debounce()
{
  // Weighted exponential smoothing for debouncing
  // (replaces capacitor between encoder and Inv. Schmitt Trigger)
  _time_debounceInterval = micros() - _time_debounceInterval;
  for (size_t i = 0; i < 3; i++) {
    smooth(_state_buffer[i], _state_raw[i], _time_debounceInterval, 0.01);
  }
  _time_debounceInterval = micros();

  // Hysteresis for debouncing (replaces Inv. Schmitt Trigger)
  for (size_t i = 0; i < 3; i++) {
    trigger(_state_raw_debounced[i], _state_buffer[i], exp(-1), 1-exp(-1));
  }
}

void Encoder::_refreshState()
{
  // Save last CLK & DT values and erase everything else
  // MÖGL. FEHLERQUELLE, WENN STATES IN ENCODER::ISR VERGL. WERDEN !!
  _state <<= 2;
  _state  &= 0b1100;

  // Write new states to _state
  _state  |=
    _state_raw_debounced[SW]  << 4 |
    _state_raw_debounced[CLK] << 1 |
    _state_raw_debounced[DT];
}

cmd_t Encoder::_getCommand(int8_t state)
{
  cmd_t cmd = IDLE;

  if (     state >> 4      ==  1) { cmd = PRESS; } else { cmd = RELEASE; }
  if (_LUT[state & 0b1111] == -1) { cmd = LEFT;  } else
  if (_LUT[state & 0b1111] ==  1) { cmd = RIGHT; }

  return cmd;
}

int8_t Encoder::getState()
{
  return _state;
}


// -------------------------- INPUT / POWERSWITCH --------------------------- //

void PowerSwitch::monitor()
{
  digitalRead(INPUT_PIN_POWER)
    ? powerOutput = true
    : powerOutput = false;
}



// =================================== GUI ================================== //

void GUI::init()
{
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextColor(WHITE);
  Settings::init();
}

void GUI::clear()
{
  display.clearDisplay();
  display.display();
}

void GUI::draw()
{
  display.clearDisplay();

  if (page == SETTINGS){
    Settings::draw();
  } else
  if (page == READOUT) {
    Readout::draw();
  } else
  if (page == DEBUG) {
    Debug::draw();
  } else
  if (page == SPLASH) {
    SplashScreen::draw();
  }

  display.display();
}

// Idles system while running function!
void GUI::showSplash(uint8_t duration, magnitude_t order)
{
  page_t pageLast = page;
  page = SPLASH;
  draw();
  Vaporizer::waitUntil(duration, order);
  page = pageLast;
};


// ---------------------------- GUI / SPLASHSCREEN -------------------------- //

void SplashScreen::draw()
{
  display.drawBitmap(0, 0, IMG_Splash, 128, 64, WHITE);
};


// ------------------------------ GUI / SETTINGS ---------------------------- //

void Settings::init()
{
  Item::add("item1",  30, 0, 100);
  Item::add("item2",   0, 0,   1);
  Item::add("Dtem3",  99, 0, 100);
  Item::add("ttem4",   1, 0,   1);
  Item::add("item5", 999, 0, 999);
}

void Settings::draw()
{
  display.drawRect(0, 0, 128, 64, WHITE);
  display.fillRect(0, 0, 128, 12, WHITE);
  display.setCursor(40, 3);
  display.setTextColor(BLACK);
  display.print("SETTINGS");
  display.setTextColor(WHITE);
  Item::list(3, 14);
}

Item::Item(String name, int value, int valueMin, int valueMax)
{
  _name     = name;
  _valueMin = valueMin;
  _value    = value;
  _valueMax = valueMax;
}

void Item::add(String name, int value, int valueMin, int valueMax)
{
  Item item(name, value, valueMin, valueMax);
  buffer.push_back(item);
}

void Item::list(uint8_t posX, uint8_t posY)
{
  uint8_t size = buffer.size();

  for (size_t i = 0; i < size; i++) {

    // Draw selector
    i == selection
      ? display.fillRect(posX -1, posY -1, 124, _height -1, WHITE),
        display.setTextColor(BLACK)
      : display.setTextColor(WHITE);

    display.setCursor (posX, posY);
    display.println   (buffer[i]._name);

    uint8_t indent = _indent(buffer[i]._value);

    display.setCursor (posX +105 +indent, posY);
    display.println   (buffer[i]._value);

    posY += _height;
  }
}

void Item::changeValue(int n)
{
  _value += n;

  _value < 0
    ? _value = 0
    : _value;
  _value > _valueMax
    ? _value = _valueMax
    : _value;
}

uint8_t Item::_indent(int value)
{
  // Indentation of values
  uint8_t temp;

  value < 10
    ? temp = 12
    : value < 100
      ? temp = 6
      : temp = 0;

  return temp;
}


// ------------------------------- GUI / READOUT ---------------------------- //

void Readout::draw()
{
  display.drawRect(0, 0, 128, 64, WHITE);
  display.setCursor(40, 28);
  display.print("READOUT");
}


// -------------------------------- GUI / DEBUG ----------------------------- //

void Debug::draw()
{
  display.setCursor(0, 0);
  display.print("cycle:  "); display.println(cycleCount );
  display.print("FPS:    "); display.println(framerate  );
  display.print("state: ");  display.println(Encoder::getState() | 0b100000, BIN);
  display.fillRect(42, 16, 6, 9, BLACK);
}



// ================================= CIRCUIT ================================ //


// ------------------------------ CIRCUIT / LED ----------------------------- //

uint16_t linearize(float input)
{
  input > PWM_RANGE
    ? input = (float)PWM_RANGE
    : input;
  input = 1.0*input*input/PWM_RANGE;
  return (uint16_t)(input + 0.5);
}
