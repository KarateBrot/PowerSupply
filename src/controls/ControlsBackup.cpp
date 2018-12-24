// #include "ControlsBackup.h"


// // ================================= Input ====================================

// std::vector<Encoder> Encoder::_buffer;

// const uint8_t Encoder::_stateMachine[7][4] = {

//   { START,    CW_BEGIN,  CCW_BEGIN, START       },
//   { CW_NEXT,  START,     CW_FINAL,  START | CW  },
//   { CW_NEXT,  CW_BEGIN,  START,     START       },
//   { CW_NEXT,  CW_BEGIN,  CW_FINAL,  START       },
//   { CCW_NEXT, START,     CCW_BEGIN, START       },
//   { CCW_NEXT, CCW_FINAL, START,     START | CCW },
//   { CCW_NEXT, CCW_FINAL, CCW_BEGIN, START       }
// };

// Encoder::Encoder(uint8_t CLK, uint8_t DT, fptr_t cw, fptr_t ccw) {

//   _pin = CLK;
//   pinMode(_pin, INPUT);
//   digitalWrite(_pin, LOW);

//   _pin2 = DT;
//   pinMode(_pin2, INPUT);
//   digitalWrite(_pin2, LOW);

//   command    = cw;
//   commandCCW = ccw;

//   _state = START;
// }

// void Encoder::_ISR() {

//   uint8_t state = 0;

//   for (Encoder enc : _buffer) {

//     state = enc.read();

//     switch (state) {
//       case CW:
//         Controls::_commands.push_back(enc.command);
//         break;
//       case CCW:
//         Controls::_commands.push_back(enc.commandCCW);
//         break;
//       default:
//         break;
//     }
//   }
// }

// uint8_t Encoder::read() {

//   // state machine naturally debounces encoder
//   uint8_t stateTemp = (digitalRead(_pin) << 1) | digitalRead(_pin2);
//   _state = _stateMachine[_state & 0xf][stateTemp];

//   return _state & 0x30;
// }

// // - - - - -

// std::vector<Button> Button::_buffer;

// Button::Button(uint8_t pin, fptr_t c) {

//   _pin = pin;

//   pinMode(_pin, INPUT);
//   digitalWrite(_pin, LOW);

//   command = c;

//   _state = digitalRead(_pin);
// }

// void Button::_ISR() {

//   uint8_t state, size;

//   size = _buffer.size();

//   for (size_t n = 0; n < size; n++) {

//     Button &btn = _buffer[n];
//     state       = btn.read();

//     // Ignore bounces (state == 0xFF)
//     if (state != 0xFF) {
//       switch (state) {
//         case DOWN:
//           Controls::_commands.push_back(btn.command);
//           break;
//         default: 
//           break;
//       }
//     }
//   }
// }

// uint8_t Button::read() {

//   _state = 0xFF;

//   // Button debouncing is a combination of (1)hardware and (2)software debouncing
//   // (1) Low-pass filter: R = 10kΩ | C = 10nF | τ = R*C = 0.0001s
//   //     ──► Capacitor (103) parallel to button and pulldown resistor at button(-)
//   // (2) In addition: easy debouncing to ignore remaining jitters
//   if ((uint32_t)(millis() - _lastRead) >= BUTTON_DEBOUNCE_TIME) {

//     _lastRead = millis();
//     _state = digitalRead(_pin);
//   }

//   return _state;
// }

// // - - - - -

// std::vector<Switch> Switch::_buffer;

// Switch::Switch(uint8_t pin, bool *ptr) {

//   _ptr = ptr;
//   _pin = pin;

//   pinMode(_pin, INPUT);
//   digitalWrite(_pin, LOW);

//   _state = digitalRead(_pin);
// }

// uint8_t Switch::read() {

//   // easy debouncing (1000/BUTTON_DEBOUNCE_TIME switch state changes per second max.)
//   if ((uint32_t)(millis() - _lastRead) >= BUTTON_DEBOUNCE_TIME) {

//     _lastRead = millis();
//     _state = digitalRead(_pin);
//     *_ptr = _state;
//   }

//   return _state;
// }

// // --------------------------------- Input -------------------------------------




// // ================================ Controls ===================================

// std::vector<fptr_t> Controls::_commands;

// Controls::Controls() {}

// void Controls::add(Encoder enc) {

//   Encoder::_buffer.push_back(enc);
//   uint8_t pin  = digitalPinToInterrupt(Encoder::_buffer.back().getPin());
//   uint8_t pin2 = digitalPinToInterrupt(Encoder::_buffer.back().getPin2());
//   attachInterrupt(pin,  Encoder::_ISR, CHANGE);
//   attachInterrupt(pin2, Encoder::_ISR, CHANGE);
// }

// void Controls::add(Button btn) {

//   Button::_buffer.push_back(btn);
//   uint8_t pin = digitalPinToInterrupt(Button::_buffer.back().getPin());
//   attachInterrupt(pin, Button::_ISR, CHANGE);
// }

// void Controls::add(Switch sw) {

//   Switch::_buffer.push_back(sw);
// }

// void Controls::update() {

//   for (Switch sw  : Switch::_buffer) { sw.read(); }
//   for (fptr_t cmd : _commands)       { cmd();     }
//   _commands.clear();
// }

// // -------------------------------- Controls -----------------------------------
