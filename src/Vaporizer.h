//############################################################################//
//                                                                            //
//     VAPORIZER.H - Library for operating custon built vaporizer based       //
//                   on ESP8266                                               //
// -------------------------------------------------------------------------- //
//                   © 2017, Jan Post                                         //
//                                                                            //
//############################################################################//

#ifndef VAPORIZER_H
#define VAPORIZER_H


// -------------------------------------------------------------------------- //
  #include <vector>                                                           //
  using namespace std;                                                        //
// -------------------------------------------------------------------------- //
  #include <Arduino.h>                                                        //
  #include <Adafruit_INA219.h>                                                //
  #include <Adafruit_MCP4725.h>                                               //
  #include <Adafruit_GFX.h>                                                   //
    #include <Fonts/FreeSans9pt7b.h>                                          //
    #include <Fonts/FreeSans12pt7b.h>                                         //
    #include <Fonts/FreeSansBold18pt7b.h>                                     //
  #include <Adafruit_SSD1306.h>                                               //
// -------------------------------------------------------------------------- //
  #include <Images/Splash.h>                                                  //
// -------------------------------------------------------------------------- //
  #define   PID_P                850.0f                                       //
  #define   PID_I                100.0f                                       //
  #define   PID_D                 80.0f                                       //
// -------------------------------------------------------------------------- //
  #define   HEATER_RES20           1.9                                        //
  #define   HEATER_RESCABLE        0.27                                       //
  #define   HEATER_MATERIAL        SS316L                                     //
// -------------------------------------------------------------------------- //
// WeMos D1 Mini PIN LAYOUT:                                                  //
// D0 -> 16, D5 -> 14, D6 -> 12, D7 -> 13, D8 -> 15                           //
// -------------------------------------------------------------------------- //
  #define   INPUT_PIN_CLK         14                                          //
  #define   INPUT_PIN_DT          12                                          //
  #define   INPUT_PIN_SW          13                                          //
  #define   INPUT_PIN_POWER       16                                          //
// -------------------------------------------------------------------------- //
  #define   PWM_RANGE             4095                                        //
  #define   PWM_FREQ              10000                                       //
// -------------------------------------------------------------------------- //
  #define   FPS_MAX               30                                          //
// -------------------------------------------------------------------------- //
  enum PIN_ID      { DT, CLK, SW };                                           //
  enum cmd_t       { IDLE, LEFT, RIGHT, PRESS, RELEASE, SELECT, LONGSELECT }; //
  enum page_t      { SETTINGS, READOUT, DEBUG, SPLASH };                      //
  enum magnitude_t { MICRO = -6, MILLI = -3, BASE = 0, KILO = 3, MEGA = 6 };  //
  enum material_t  { SS316L, SS316, SS304 };                                  //
// -------------------------------------------------------------------------- //


// =============================== VAPORIZER ================================ //


class Vaporizer
{
  static unsigned long   _time_lastWaitCall;
  static unsigned long   _time_lastCycle;

  protected:
    static unsigned long  cycleCount;
    static          float framerate;

  public:
    static bool powerOutput;

    static void   init(void);
    static void   monitor(void);
    static void   waitUntil(unsigned long, magnitude_t);
    static void   limitFPS(uint8_t);
    static void   getFPS(void);
    static void   smooth(double &, double, unsigned long, unsigned long);
    static bool   trigger(bool &, double, float, float);
};



// --------------------------------- INPUT ---------------------------------- //

// Input Interface
class Input : public Vaporizer
{
  public:
    static void init(void);
    static void monitor(void);
    static void ISR_CLK(void);
    static void ISR_DT(void);
    static void ISR_SW(void);
    static void execute(cmd_t);
};

class Encoder : public Input
{
  static       bool          _state_raw[3];
  static       double        _state_buffer[3];
  static       unsigned long _time_debounceInterval[3];
  static       bool          _state_debounced[3];
  static       int8_t        _state;
  static const int8_t        _LUT[16];

  // static void _debounce(void);
  static void  _refreshState(PIN_ID);
  static cmd_t _getCommand(int8_t);

  public:
    static void   init(void);
    static void   monitor(void);
    static void   ISR(PIN_ID);
    static void   read(PIN_ID);
    static int8_t getState(void);
    static int8_t getLUT(uint8_t);
};

class PowerSwitch : public Input
{
  public:
    static void monitor(void);
};


// ----------------------------------- GUI ---------------------------------- //

// Graphical User Interface
class GUI : public Vaporizer
{
  public:
    static Adafruit_SSD1306 display;
    static page_t           page;

    static void init (void);
    static void clear(void);
    static void draw (void);
    static void showSplash(uint8_t, magnitude_t);
};

class SplashScreen : public GUI
{
  public:
    static void draw(void);
};

class Readout : public GUI
{
  public:
    static void draw(void);
};

class Debug : public GUI
{
  public:
    static void draw(void);
};

class Settings : public GUI
{
  public:
    static uint8_t selection;
    static bool    selected;

    static void init(void);
    static void draw(void);
};

class Item : public Settings
{
  static const uint8_t _height = 10;

  String _name;
  int    _value, _valueMin, _valueMax;

  static uint8_t _indent(int);

  public:
    static vector<Item>  buffer;

    Item(String, int, int, int);
    static void add(String, int, int, int);
    static void list(uint8_t, uint8_t);
    void        changeValue(int);
};


// -------------------------------- REGULATION ------------------------------ //

// Circuit Regulation Interface
class Circuit : public Vaporizer
{

};

class PID : public Circuit
{

};

class Heater : public Circuit
{

};

class LED : public Circuit
{
  public:
    static uint16_t linearize(float);
};



// ========================================================================== //


#endif // VAPORIZER_H
