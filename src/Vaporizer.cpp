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




  // ================================ TIMER ================================= //

  Timer::Timer() {

    _time         = micros();
    _lastWaitCall = micros();
    _lastCycle    = micros();
  }

  void Timer::waitUntil(uint32_t timer) {

    while (micros() - _lastWaitCall < timer) { yield(); }
    _lastWaitCall = micros();
  }

  void Timer::limitFPS(uint8_t fps) {

    waitUntil( (uint32_t)(1000000.0f/fps + 0.5f) );
  }

  float Timer::getFPS() {

    float framerate = 1000000.0f/(micros() - _lastCycle);
    _lastCycle = micros();
    return framerate;
  }

  // -------------------------------- TIMER --------------------------------- //




  // ================================ SENSOR ================================ //

  Sensor::Sensor() {

    _INA219.begin();
    _INA219.setCalibration_32V_2A();
  }

  void Vaporizer::Sensor::setPrecision(bool b) {

    b ? _INA219.setCalibration_16V_400mA() : _INA219.setCalibration_32V_2A();
  }

  void Vaporizer::Sensor::read() {

    // Current [mA]
    current = (double)_INA219.getCurrent_mA()*2.0;   // R050 instead of R100 shunt

    // Voltage [mV]
    voltage = (double)_INA219.getBusVoltage_V()*1000.0;
  }

  // ------------------------------- SENSOR --------------------------------- //




  // ================================= DAC ================================== //

  DAC::DAC() {

    _MCP4725.begin(0x62);
    _MCP4725.setVoltage(4095, false);            // boot with minimal power output
  }

  // Applies DC voltage from 0..Vcc at pin "OUT" on MCP4725 breakout board
  void Vaporizer::DAC::setOutput(uint16_t val) {

    _MCP4725.setVoltage(4095 - val, false);
  }

  // --------------------------------- DAC ---------------------------------- //




  // =============================== PID_Ctrl =============================== //

  PID_Ctrl::PID_Ctrl() {

  }

  double PID_Ctrl::getOutput(double value, double value_set) {

    // ---------------------------------------------------------------------- //
    //                        --< PID-CONTROLLER >--                          //
    // -----------------------------------------------------------------------//
    // SIGNAL OUTPUT:  u(t) = P*e(t) + I*∫e(t)dt + D*de(t)/dt                 //
    //                     e(t)     = ΔT                                      //
    //                     ∫e(t)dt  = idle                                    //
    //                     de(t)/dt = -dT/dt                                  //
    // -----------------------------------------------------------------------//
    //             =>  u(t) = P*ΔT + I*idle - D*dT/dt                         //
    // ---------------------------------------------------------------------- //

    // e(t)
    _error = value_set - value;

    // dt
    _dt       = (micros() - _timeLast)/1000000.0;
    _timeLast =  micros();

    // de(t)/dt
    _errorDiff = (value - _valueLast)/_dt;
    _valueLast =  value;

    // ∫e(t)dt
    if (_error  <= 0.025) {                                                     // only start integrating shortly before reaching e(t) = 0 (to prevent integral windup)
      _errorInt += _error*_dt + 0.001;                                          // [+ 0.001]: let P and I fight each other (for "stiffer" temp regulation)
      _errorInt  = constrain(_errorInt, 0, 1);
    }

    // u(t) = P*e(t) + I*∫e(t)dt + D*de(t)/dt
    return constrain(_p*_error + _i*_errorInt - _d*_errorDiff, 0, 1);
  }

  void PID_Ctrl::setOutputOfDAC(DAC &d, double value, double value_set) {

    double output = getOutput(value, value_set)*4095.0;
    d.setOutput((uint16_t)output);
  }

  void PID_Ctrl::autotune() {


  }

  // ------------------------------- PID_Ctrl ------------------------------- //




  // ================================ Heater ================================ //

  Heater::Heater() {

    setTCR     (HEATER_TCR_SS316L);
    setRes20   (HEATER_RES20);
    setResCable(HEATER_RESCABLE);
    pid.setPID (PID_P, PID_I, PID_D);
  }

  void Heater::update() {

    sensor.read();

    // Resistance [Ω]
    resistance =
      0.9*resistance +
      0.1*(sensor.voltage/constrain(sensor.current, 1, 15000) - _resCable);

    // Resistance [Ω] - if no heater connected
    if (sensor.voltage > 100 && sensor.current < 10) { resistance = _res20; }

    // Power [W]
    power =
      0.5*power +
      0.5*sensor.voltage*sensor.current/1000000.0;

    // Temperature [°C]
    temperature =
      0.95*temperature +
      0.05*(_TCR*log(resistance/_res20) + 20.0);
  }

  // Sets DAC pin "OUT" to DC voltage according to PID-controller
  void Heater::regulate() {

    pid.setOutputOfDAC(dac, temperature, temperature_set);
  }

  // Make sure heater core is at room temperature before calibration!
  void Heater::calibrate() {

    PID_Ctrl pid_helper;
    pid_helper.setPID(1.0, 0.0, 0.0);               // still needs manual tuning

    sensor.setPrecision(HIGH);

    // Make sure current flow is 10mA +/- 1mA
    double currentLast = 0.0;
    while (abs(sensor.current - 10.0) > 1.0 && abs(currentLast - 10.0) > 1.0) {
      currentLast = sensor.current;
      update();
      pid_helper.setOutputOfDAC(dac, sensor.current, 10.0);
    }

    // Calculate resistance using reference current of 10mA
    for (size_t i = 0; i < 15; i++) {
      update();
      pid_helper.setOutputOfDAC(dac, sensor.current, 10.0);
    }

    dac.setOutput(0);
    setRes20(resistance);
    sensor.setPrecision(LOW);
  }

  // -------------------------------- Heater -------------------------------- //




  // ================================= GUI ================================== //

  GUI::GUI() {

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.setTextColor(WHITE);
  }

  void GUI::clear() {

    display.clearDisplay();
    display.display();
  }

  // --------------------------------- GUI ---------------------------------- //




  // ============================== VAPORIZER =============================== //

  Timer  timer;
  Heater heater;
  Input  input;
  GUI    gui;

  void init(uint8_t scl, uint8_t sda) {

    Wire.begin(scl, sda);                  // Select I2C pins
    Wire.setClock(800000L);                // for faster I2C transmission (800kHz)
  }

  // ------------------------------ VAPORIZER ------------------------------- //




}
