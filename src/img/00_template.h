// ###################### //
// #                    # //
// #    ANI_template    # //
// #                    # //
// #--------------------# //
// #   Width:       x   # //
// #   Height:      y   # //
// ###################### //


#ifndef ANI_template_h
#define ANI_template_h


#include "Arduino.h"

// FRAMES
static const uint8_t PROGMEM IMG_template_00[] = {};
static const uint8_t PROGMEM IMG_template_01[] = {};
static const uint8_t PROGMEM IMG_template_02[] = {};
static const uint8_t PROGMEM IMG_template_03[] = {};
static const uint8_t PROGMEM IMG_template_04[] = {};
// etc...

// ANIMATION (pointer to frames)
const uint8_t *ANI_template[] = {

  IMG_template_00,
  IMG_template_01,
  IMG_template_02,
  IMG_template_03,
  IMG_template_04
};


#endif // ANI_template_h
