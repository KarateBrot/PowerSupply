//############################################################################//
//                                                                            //
//     VAPORIZER.CPP - Library for operating custon built vaporizer based     //
//                     on ESP8266                                             //
// -------------------------------------------------------------------------- //
//                     © 2018, Jan Post                                       //
//                                                                            //
//############################################################################//


#include <Vaporizer.h>




namespace Vaporizer {




  // ================================ TIMER ====================================

  Timer::Timer() {

    _time         = micros();
    _lastWaitCall = micros();
    _lastCycle    = micros();
  }

  void Timer::waitUntil(uint32_t timer) {

    while (micros() - _lastWaitCall < timer) { yield(); }
    _lastWaitCall = micros();
  }

  void Timer::limitCPS(uint8_t fps) {

    waitUntil( (uint32_t)(1000000.0f/fps + 0.5f) );
  }

  float Timer::getCPS() {

    float cyclespersecond = 1000000.0f/(micros() - _lastCycle);
    _lastCycle = micros();
    return cyclespersecond;
  }

  // -------------------------------- TIMER ------------------------------------




  // ================================ SENSOR ===================================

  Adafruit_INA219 Sensor_Power::_INA219;

  Sensor_Power::Sensor_Power() {

    _INA219.begin();
  }

  void Sensor_Power::setPrecision(bool b) {

    b ? _INA219.setCalibration_16V_400mA() : _INA219.setCalibration_32V_2A();
  }

  void Sensor_Power::read() {

    // Current [mA]
    current =
      0.5*current +
      0.5*_INA219.getCurrent_mA()*2.0;             // R050 instead of R100 shunt

    // Voltage [mV]
    voltage =
      0.5*voltage +
      0.5*_INA219.getBusVoltage_V()*1000.0;
  }

  // ------------------------------- SENSOR ------------------------------------




  // ================================= DAC =====================================

  Adafruit_MCP4725 DAC::_MCP4725;

  DAC::DAC() {

    _MCP4725.begin(0x62);
    _MCP4725.setVoltage(4095, false);          // boot with minimal power output
  }

  // Applies DC voltage from 0..Vcc at pin "OUT" on MCP4725 breakout board
  void DAC::setOutput(uint16_t val) {

    _MCP4725.setVoltage(4095 - val, false);
  }

  // --------------------------------- DAC -------------------------------------




  // =============================== PID_Ctrl ==================================

  // ------------------------------------------------------------------------ //
  //                         --< PID-CONTROLLER >--                           //
  // -----------------:------------------------------------------------------ //
  // SIGNAL OUTPUT:   : u(t) = P*e(t) + I*∫e(t)dt + D*de(t)/dt                //
  // -----------------:------------------------------------------------------ //
  // ERROR:           :     e(t)     = T_set - T(t) = ΔT                      //
  // PAST ERR:        :     ∫e(t)dt  = e_past                                 //
  // PREDICTED ERR:   :     de(t)/dt = -dT/dt                                 //
  // -----------------:------------------------------------------------------ //
  //       ──►        : u(t) = P*ΔT + I*e_past - D*dT/dt                      //
  // -----------------:------------------------------------------------------ //

  PID_Ctrl::PID_Ctrl() {

  }

  void PID_Ctrl::_update(double value, double value_set) {

    // e(t)
    _error = value_set - value;

    // dt
    _dt       = (micros() - _timeLast)/1000000.0;
    _timeLast =  micros();

    // de(t)/dt
    _errorDiff = (value - _valueLast)/_dt;
    _valueLast =  value;

    // ∫e(t)dt
    if (_error  <= 10) {      // only integrate shortly before reaching e(t) = 0
      _errorInt += _error*_dt;
      _errorInt  = constrain(_errorInt, 0, 1/_i);
    }
  }

  vector<double> PID_Ctrl::getPID() const {

    vector<double> v({_p, _i, _d});
    return v;
  }

  double PID_Ctrl::getOutput() const {

    // u(t) = P*ΔT + I*e_past - D*dT/dt
    return constrain(_p*_error + _i*_errorInt - _d*_errorDiff, 0, 1);
  }

  void PID_Ctrl::regulate(double value, double value_set) {

    _update(value, value_set);
    uint16_t output = (uint16_t)( getOutput()*4095.0 + 0.5 );
    _dacPointer->setOutput(output);
  }

  void PID_Ctrl::autotune() {

    // TODO: Autotune algorithm
  }

  // ------------------------------- PID_Ctrl ----------------------------------




  // ================================ Heater ===================================

  Heater::Heater() {

    setTCR     (HEATER_TCR);
    setRes20   (HEATER_RES20);
    setResCable(HEATER_RESCABLE);

    resistance  = HEATER_RES20;
    temperature = 20.0;

    pid
      .attach(&dac)
      .setPID(PID_P, PID_I, PID_D);
  }

  void Heater::update() {

    sensor.read();

    // Resistance [Ω]
    resistance =
      0.7*resistance +
      0.3*(sensor.voltage/constrain(sensor.current, 1, 15000) - _resCable);

    // Resistance [Ω] - if no heater connected
    if (sensor.voltage > 100 && sensor.current < 10) { resistance = _res20; }

    // Power [W]
    power =
      0.5*power +
      0.5*sensor.voltage*sensor.current/1000000.0;

    // Temperature [°C]
    temperature =
      0.7*temperature +
      0.3*(log(resistance/_res20)/_TCR + 20.0);
  }

  // Sets DAC pin "OUT" to DC voltage according to PID-controller
  void Heater::regulate() {

    if (state == ON) {

      // TODO: Set ideal PID consts for each mode
      mode == TEMP_MODE
        ? pid.regulate(temperature, temperature_set)
        : pid.regulate(power, power_set);
      sensor.setPrecision(LOW);

    } else {

      dac.setOutput(0);
      sensor.setPrecision(HIGH);
    }
  }

  // Make sure heater core is at room temperature before calibration!
  void Heater::calibrate() {

    PID_Ctrl pid_calibration;

    pid_calibration
      .attach(&dac)
      .setPID(1.0, 0.0, 0.0);                             // TODO: Manual tuning

    sensor.setPrecision(HIGH);

    // Make sure current flow is 10mA +/- 1mA
    double currentLast = 0.0;
    while (abs(sensor.current - 10.0) > 1.0 && abs(currentLast - 10.0) > 1.0) {
      currentLast = sensor.current;
      update();
      pid_calibration.regulate(sensor.current, 10.0);
      yield();
    }

    // Calculate resistance using reference current of 10mA
    for (size_t i = 0; i < 15; i++) {
      update();
      pid_calibration.regulate(sensor.current, 10.0);
      yield();
    }

    // TODO: Room temp measurement to compensate res for temps != 20°C

    dac.setOutput(0);
    setRes20(resistance);
    sensor.setPrecision(LOW);
  }

  // -------------------------------- Heater -----------------------------------




  // =============================== CONTROLS ==================================

  const uint8_t Encoder::_LUT[7][4] = {

    { START,    CW_BEGIN,  CCW_BEGIN, START       },
    { CW_NEXT,  START,     CW_FINAL,  START | CW  },
    { CW_NEXT,  CW_BEGIN,  START,     START       },
    { CW_NEXT,  CW_BEGIN,  CW_FINAL,  START       },
    { CCW_NEXT, START,     CCW_BEGIN, START       },
    { CCW_NEXT, CCW_FINAL, START,     START | CCW },
    { CCW_NEXT, CCW_FINAL, CCW_BEGIN, START       }
  };

  Encoder::Encoder() {

    _state = START;
  }

  void Encoder::begin(uint8_t pinCLK, uint8_t pinDT) {

    _pinCLK = pinCLK;
    _pinDT  = pinDT;

    pinMode(_pinCLK, INPUT);
    pinMode(_pinDT,  INPUT);

    digitalWrite(_pinCLK, HIGH);
    digitalWrite(_pinDT,  HIGH);
  }

  uint8_t Encoder::read() {

    uint8_t stateTemp = (digitalRead(_pinCLK) << 1) | digitalRead(_pinDT);
    _state = _LUT[_state & 0xf][stateTemp];

    return _state & 0x30;
  }

  // ------------------------------- CONTROLS ----------------------------------




  // ================================ Input ====================================

  Encoder Input::encoder;

  Input::Input() {


  }

  void Input::_isr_encoder() {

    uint8_t state = encoder.read();

    switch (state) {
      case Encoder::CW:  Serial.println("Right"); break;
      case Encoder::CCW: Serial.println("Leftt"); break;
      default:           break;
    }
  }

  void Input::addEncoder(uint8_t pinCLK, uint8_t pinDT) {

    encoder.begin(pinCLK, pinDT);

    attachInterrupt(pinCLK, _isr_encoder, CHANGE);
    attachInterrupt(pinDT,  _isr_encoder, CHANGE);
  }

  void Input::addButton(uint8_t pin, void *isr()) {

    buttons.push_back(Button());
  }

  void Input::addSwitch(uint8_t pin, void *isr()) {

    switches.push_back(Switch());
  }

  void Input::update() {

    // TODO: process/execute action_t stack
  }

  // -------------------------------- Input ------------------------------------




  // ================================= GUI =====================================

  GUI::GUI() {

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.setTextColor(WHITE);
  }

  void GUI::clear() {

    display.clearDisplay();
    display.display();
  }

  // --------------------------------- GUI -------------------------------------




  // ============================== VAPORIZER ==================================

  Heater heater;
  Input  input;
  GUI    gui;

  // Needs to be called lastly in Setup() to overwrite Wire (I2C) settings
  void init(uint8_t scl, uint8_t sda) {

    Wire.begin(sda, scl);                // Select I2C pins
    Wire.setClock(800000L);              // for faster I2C transmission (800kHz)
  }

  // ------------------------------ VAPORIZER ----------------------------------




}
