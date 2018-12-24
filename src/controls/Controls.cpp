#include "Controls.h"


std::vector<Event>  Controls::_events;


void Controls::update() {

  for (Event e : _events) { 
    
    switch (e) {

      case Event::UP:
        break;

      case Event::DOWN:
        break;

      case Event::SELECT:
        break;

      case Event::HOLD:
        break;
        
      default:
        break;
    }
  }
  _events.clear();
}


std::vector<cptr_t> Button::_buffer;


Button::Button(uint8_t pin, Event press, Event longpress) {

  _pins.push_back(pin);
  
  _event1 = press;
  _event2 = longpress;

  pinMode(pin, INPUT_PULLUP);
  digitalWrite(pin, HIGH);

  attachInterrupt(pin, _isr, CHANGE);

  _buffer.emplace_back(shared_from_this());
}


void Button::_isr() {

  for(cptr_t btn : _buffer)
  {
    switch(btn->read()) {

      case RELEASED:
        //_events.emplace_back(_event1);
        break;

      case RELEASED_DELAY:
        //_events.emplace_back(_event2);
        break;

      default:
        break;
    }
  }
}


uint8_t Button::read() {

  // Button debouncing is combination of (1)hardware and (2)software debouncing
  // (1) Low-pass filter: R = 10kΩ | C = 10nF | τ = R*C = 0.0001s
  //     ──► Capacitor (103) parallel to button and pulldown resistor at button(-)
  // (2) In addition: easy debouncing to ignore remaining jitters
  uint32_t timer = micros();
  if ((uint32_t)(timer - _lastRead) > BUTTON_DEBOUNCE_MICROS) {
    
    _lastRead  = timer;
    _lastState = _state;
    _state     = digitalRead(_pins.back());

    // Check if interrupt change happened, else return 0xff
    if (_state != _lastState) { 
      
      switch (_state) {
      
        case PRESSED:
          _lastPressed = timer;
          break;

        case RELEASED:
          if ((uint32_t)(timer - _lastPressed) > BUTTON_DELAY_MICROS) {
            return RELEASED_DELAY;
          } 
          break;

        default:
          break;
      }
      
      return _state; 
    }
  }

  return 0xff;
}


std::vector<cptr_t> Encoder::_buffer;

const uint8_t Encoder::_stateMachine[7][4] = {

  { START,    CW_BEGIN,  CCW_BEGIN, START       },
  { CW_NEXT,  START,     CW_FINAL,  START | CW  },
  { CW_NEXT,  CW_BEGIN,  START,     START       },
  { CW_NEXT,  CW_BEGIN,  CW_FINAL,  START       },
  { CCW_NEXT, START,     CCW_BEGIN, START       },
  { CCW_NEXT, CCW_FINAL, START,     START | CCW },
  { CCW_NEXT, CCW_FINAL, CCW_BEGIN, START       }
};


Encoder::Encoder() {

  
}


void Encoder::_isr() {

  for(cptr_t enc : _buffer)
  {
    switch(enc->read()) {

      case CW:
        //_events.emplace_back(_event2);
        break;

      case CCW:
        //_events.emplace_back(_event1);
        break;

      default:
        break;
    }
  }
}


uint8_t Encoder::read() {

  // state machine naturally debounces encoder
  uint8_t stateTemp = (digitalRead(_pins.front()) << 1) | digitalRead(_pins.back());
  _state = _stateMachine[_state & 0xf][stateTemp];

  return _state & 0x30;
}