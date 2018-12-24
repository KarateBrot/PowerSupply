// #ifndef CONTROLS_BACKUP_H
// #define CONTROLS_BACKUP_H


// #include "Arduino.h"
// #include <vector>

// typedef void (*fptr_t)(void);

// #define BUTTON_DEBOUNCE_TIME   50                     // Debounce interval in ms




// // =================================== INPUT ===================================

// struct Input {
  
//   friend class Controls;

// protected:
//   uint8_t _state, _pin;

// public:
//   enum state_t { UP, DOWN };
  
//   fptr_t command;

//   uint8_t getPin(void) const { return _pin; }

//   virtual uint8_t read(void) = 0;
// };

// struct Encoder : public Input {

//   friend class Controls;

// private:
//   static std::vector<Encoder> _buffer;
//   static const uint8_t        _stateMachine[7][4];
//   uint8_t                     _pin2;

//   static void _ISR(void);

// public:
//   enum state_t {
//     START, CW_FINAL, CW_BEGIN, CW_NEXT, CCW_BEGIN, CCW_FINAL, CCW_NEXT,
//     CW = 0x10, CCW = 0x20
//   };

//   fptr_t commandCCW;

//   Encoder(uint8_t, uint8_t, fptr_t, fptr_t);

//   uint8_t getPin2(void) const { return _pin2; }
//   uint8_t read   (void);
// };

// struct Button : public Input {

//   friend class Controls;

// private:
//   static std::vector<Button> _buffer;
//   uint32_t                   _lastRead;

//   static void _ISR(void);

// public:
//   Button(uint8_t, fptr_t);

//   uint8_t read(void);
// };

// struct Switch : public Input {

//   friend class Controls;

// private:
//   static std::vector<Switch> _buffer;
//   bool                      *_ptr;
//   uint32_t                   _lastRead;

// public:
//   Switch(uint8_t, bool*);

//   uint8_t read(void);
// };

// // ----------------------------------- INPUT -----------------------------------




// // ================================= CONTROLS ==================================

// class Controls {

//   friend class Encoder;
//   friend class Button;
//   friend class Switch;

// private:
//   static std::vector<fptr_t> _commands;

// public:
//   Controls(void);

//   static void add(Encoder);
//   static void add(Button);
//   static void add(Switch);

//   static void update(void);
// };

// // --------------------------------- CONTROLS ----------------------------------




// #endif // CONTROLS_H
