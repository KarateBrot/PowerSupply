//############################################################################//
//                                                                            //
//       VARINIT.CPP - Static variable initialization for library for         //
//                     operating custon built vaporizer based on ESP8266      //
// -------------------------------------------------------------------------- //
//                     © 2017, Jan Post                                       //
//                                                                            //
//############################################################################//


#include <Vaporizer.h>



// ========================== VARIABLES & OBJECTS =========================== //


// --- PRIVATE STATIC VAR INIT ---

  unsigned long Vaporizer:: _time_lastWaitCall        = 0;
  unsigned long Vaporizer:: _time_lastCycle           = 0;

  bool          Encoder::   _state_raw[3]             = { 0, 0, 0 };
  double        Encoder::   _state_buffer[3]          = { 0, 0, 0 };
  unsigned long Encoder::   _time_debounceInterval[3] = { 0, 0, 0 };
  bool          Encoder::   _state_debounced[3]       = { 0, 0, 0 };
  int8_t        Encoder::   _state                    = 0b0;
  int8_t const  Encoder::   _LUT[16]                  = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };


// --- PUBLIC  STATIC VAR INIT ---

  unsigned long    Vaporizer:: cycleCount  = 0;
  float            Vaporizer:: framerate   = 0;

  Adafruit_SSD1306 GUI::       display;
  page_t           GUI::       page        = DEBUG;
  uint8_t          Settings::  selection   = 0;
  bool             Settings::  selected    = 0;
  vector<Item>     Item::      buffer;

  bool             Vaporizer:: powerOutput = 0;


// ========================================================================== //
