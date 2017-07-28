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
bool Vaporizer::trigger(bool &trigger, double val, float lim_lower, float lim_upper)
{
  bool trigger_last = trigger;

  if (val >= lim_upper) { trigger = true;  } else
  if (val <  lim_lower) { trigger = false; }

  if (trigger_last != trigger) { return true; } else { return false; }
}



// ================================== INPUT ================================= //

void Input::init()
{
  pinMode(INPUT_PIN_CLK,   INPUT);
  pinMode(INPUT_PIN_DT,    INPUT);
  pinMode(INPUT_PIN_SW,    INPUT);
  pinMode(INPUT_PIN_POWER, INPUT);

  attachInterrupt(INPUT_PIN_CLK, ISR_CLK, CHANGE);
  attachInterrupt(INPUT_PIN_DT,  ISR_DT,  CHANGE);
  attachInterrupt(INPUT_PIN_SW,  ISR_SW,  CHANGE);

  Encoder::init();
}

void Input::monitor()
{
  Encoder::monitor();
  PowerSwitch::monitor();
}

// Needs Input::monitor() to be looped to function properly
void Input::ISR_CLK() { Encoder::read(CLK); }
void Input::ISR_DT()  { Encoder::read(DT);  }
void Input::ISR_SW()  { Encoder::read(SW);  }

void Input::execute(cmd_t command)
{
  switch (command)
  {
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
  for (size_t i = 0; i < 3; i++) { read(CLK); read(DT); read(SW); }
}

void Encoder::monitor()
{
  read(CLK);
  read(DT);
  read(SW);
}

void Encoder::read(PIN_ID pin)
{
  switch (pin)
  {
    case CLK: _state_raw[pin] =  digitalRead(INPUT_PIN_CLK); break;
    case DT:  _state_raw[pin] =  digitalRead(INPUT_PIN_DT ); break;
    case SW:  _state_raw[pin] = !digitalRead(INPUT_PIN_SW ); break;
  }

  _time_debounceInterval[pin] = micros() - _time_debounceInterval[pin];
  smooth(_state_buffer[pin], _state_raw[pin], _time_debounceInterval[pin], 10000);
  _time_debounceInterval[pin] = micros();

  bool triggered = trigger(_state_debounced[pin], _state_buffer[pin], exp(-1), 1-exp(-1));

  if (triggered) { _refreshState(pin); }
}

// void Encoder::_debounce()
// {
//
// }

void Encoder::_refreshState(PIN_ID pin)
{
  switch (pin)
  {
    case CLK:
      _state &= 0b10111;
      _state |= (_state & 0b10) << 2;
      _state &= 0b11101;
      _state |= _state_debounced[pin] << 1;
      break;
    case DT:
      _state &= 0b11011;
      _state |= (_state & 0b1) << 2;
      _state &= 0b11110;
      _state |= _state_debounced[pin];
      break;
    case SW:
      _state &= 0b01111;
      _state |= _state_debounced[pin] << 4;
      break;
  }
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

int8_t Encoder::getLUT(uint8_t num)
{
  return _LUT[num];
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
  display.print("state: ");  display.print  (Encoder::getState() | 0b100000, BIN);
  display.fillRect(42, 16, 6, 9, BLACK);
  display.print(" [");  display.print(Encoder::getLUT(Encoder::getState() & 0b1111)); display.println("]");
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
