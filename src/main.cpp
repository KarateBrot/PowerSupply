
#include <Vaporizer.h>
  using namespace Vaporizer;

#include <MegunoLink.h>
#include <CommandHandler.h>


#define SCL   5                                                        // Pin D1
#define SDA   4                                                        // Pin D2


Timer  timer;
Table  properties;
XYPlot plot_temp("Temp"), plot_power("Power");
CommandHandler<> cmd;


void sendData() {

  properties.SendData("- - - - - - - - - - - - - -", "- - MAIN - -", "- - - - - - - - - - - - - -");
  properties.SendData("Temperature",     heater.temperature,     "degC");
  properties.SendData("Power",           heater.power,           "W"   );
  properties.SendData("Temperature_set", heater.temperature_set, "degC");
  properties.SendData("Power_set",       heater.power_set,       "W"   );
  properties.SendData("Cycles/s",        timer.getCPS(),         "Hz"  );
  properties.SendData("Heater Status",   heater.state );

  plot_temp.SendData("Temperature", timer.cycle, heater.temperature, Plot::Red);
  plot_power.SendData("Power",      timer.cycle, heater.power,       Plot::Blue);
}

void Cmd_setTemperature(CommandParameter &Parameters) {

  heater.temperature_set = Parameters.NextParameterAsInteger();
}

void Cmd_setPower(CommandParameter &Parameters) {

  heater.power_set = Parameters.NextParameterAsInteger();
}

void Cmd_toggleHeater(CommandParameter &Parameters) {

  heater.state == OFF ? heater.on() : heater.off();
}


// ================================== RUN =================================== //

void setup() {

  Serial.begin(115200);
  init(SCL, SDA);
  cmd.AddCommand(F("setTemperature"), Cmd_setTemperature);
  cmd.AddCommand(F("setPower"),       Cmd_setPower      );
  cmd.AddCommand(F("heaterToggle"),   Cmd_toggleHeater  );
}

void loop() {

  heater.update();
  heater.regulate();
  sendData();
  cmd.Process();
  timer.limitCPS(30);
  timer.cycle++;
}

// ---------------------------------- RUN ----------------------------------- //
