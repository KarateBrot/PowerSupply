#ifndef CONTROLS_H
#define CONTROLS_H

#include "Arduino.h"
#include <memory>
#include <vector>

typedef void(*fptr_t)(void);

class Controls;
typedef std::shared_ptr<Controls> cptr_t;


#define BUTTON_DEBOUNCE_MICROS 50000
#define BUTTON_DELAY_MICROS    1000000




enum class Event : uint8_t {

  NONE   = 0,
  UP     = 0b1, 
  DOWN   = 0b10, 
  SELECT = 0b100, 
  HOLD   = 0b1000
};


class Controls : public std::enable_shared_from_this<Controls> {

protected:
  static std::vector<Event>  _events;

  std::vector<uint8_t> _pins;
  Event                _event1, _event2;
  volatile uint8_t     _state;

public:
  static void update(void);

  std::vector<uint8_t> getPins  (void) const { return _pins;   }
  Event                getEvent1(void) const { return _event1; }
  Event                getEvent2(void) const { return _event2; }

  virtual uint8_t read(void) = 0;
};


class Button : public Controls {

private:
  static std::vector<cptr_t> _buffer;
  
  volatile uint8_t  _lastState;
  volatile uint32_t _lastRead, _lastPressed;
  
  static void _isr(void);

public:
  Button(uint8_t pin, Event press, Event longpress);

  enum state_t : uint8_t { 
    PRESSED, 
    RELEASED,
    RELEASED_DELAY
  };

  uint8_t read(void) override;
};


class Encoder : public Controls {

private:
  static std::vector<cptr_t> _buffer;
  
  static const uint8_t _stateMachine[7][4];
  
  static void _isr(void);

public:
  Encoder(void);

  enum state_t : uint8_t {
    START, 
    CW_FINAL,  CW_BEGIN,  CW_NEXT,
    CCW_BEGIN, CCW_FINAL, CCW_NEXT,
    CW = 0x10, CCW = 0x20
  };

  uint8_t read(void) override;
};


#endif // CONTROLS_H