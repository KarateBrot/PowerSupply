
#include <Vaporizer.h>
  using namespace Vaporizer;

#include <MegunoLink.h>
#include <CommandHandler.h>


#define SCL   5                                                        // Pin D1
#define SDA   4                                                        // Pin D2


Timer          timer;
Table          table;
Message        msg;
InterfacePanel panel;

TimePlot plot_temp("Temp"), plot_power("Power");

CommandHandler<> cmd;


void sendData() {

  table.SendData("- - MAIN - - - - - -", "");

  table.SendData("Current",  heater.sensor.current/1000.0, "A" );
  table.SendData("Voltage",  heater.sensor.voltage/1000.0, "V" );
  table.SendData("Cycles/s", timer.getCPS(),               "Hz");
  table.SendData("Cycle",    timer.counter                     );

  table.SendData("- - HEATER - - -", "");

  table.SendData("Resistance",  heater.resistance,  "Ohm");
  table.SendData("Power",       heater.power,       "W"  );
  table.SendData("Temperature", (int)(heater.temperature + 0.5), "*C" );

  heater.state == ON
    ? table.SendData("State", "ON")
    : table.SendData("State", "OFF");
  heater.mode == TEMP_MODE
    ? table.SendData("Mode", "TEMP")
    : table.SendData("Mode", "POWER");

  table.SendData("- - PID - - - - - - -", "");

  table.SendData("K_p", heater.pid.getPID()[0]);
  table.SendData("K_i", heater.pid.getPID()[1]);
  table.SendData("K_d", heater.pid.getPID()[2]);

  plot_temp.SendData ("Temperature",     heater.temperature,     Plot::Red,   Plot::Solid, 2, Plot::NoMarker);
  plot_temp.SendData ("Temperature_set", heater.temperature_set, Plot::Black, Plot::Solid, 1, Plot::NoMarker);

  plot_power.SendData("Power",           heater.power,           Plot::Red,   Plot::Solid, 2, Plot::NoMarker);
  plot_power.SendData("Power_set",       heater.power_set,       Plot::Black, Plot::Solid, 1, Plot::NoMarker);
  plot_power.SendData("PID_Output", 10.0*heater.pid.getOutput(), Plot::Cyan,  Plot::Solid, 1, Plot::NoMarker);
}

void Cmd_setTemperature(CommandParameter &Parameters) {

  heater.temperature_set = Parameters.NextParameterAsInteger();
}

void Cmd_setPower(CommandParameter &Parameters) {

  heater.power_set = Parameters.NextParameterAsInteger();
}

void Cmd_toggleHeater(CommandParameter &Parameters) {

  heater.state == OFF ? heater.on() : heater.off();

  if (heater.state == ON) {
    msg.Send("Heater ON");
  } else {
    msg.Send("Heater OFF");
  }
}

void Cmd_setMode(CommandParameter &Parameters) {

  uint8_t temp = Parameters.NextParameterAsInteger();
  temp == 0 ? heater.setMode(TEMP_MODE) : heater.setMode(POWER_MODE);
  temp == 0 ? msg.Send("Heater TEMP_MODE") : msg.Send("Heater POWER_MODE");
}

void Cmd_setP(CommandParameter &Parameters) {

  heater.pid.setP(Parameters.NextParameterAsDouble());
}

void Cmd_setI(CommandParameter &Parameters) {

  heater.pid.setI(Parameters.NextParameterAsDouble());
}

void Cmd_setD(CommandParameter &Parameters) {

  heater.pid.setD(Parameters.NextParameterAsDouble());
}


// ================================== RUN =================================== //

void setup() {

  Serial.begin(500000);
  init(SCL, SDA);

  cmd.AddCommand(F("setTemperature"), Cmd_setTemperature);
  cmd.AddCommand(F("setPower"),       Cmd_setPower      );
  cmd.AddCommand(F("heaterToggle"),   Cmd_toggleHeater  );
  cmd.AddCommand(F("setMode"),        Cmd_setMode       );
  cmd.AddCommand(F("setP"),           Cmd_setP          );
  cmd.AddCommand(F("setI"),           Cmd_setI          );
  cmd.AddCommand(F("setD"),           Cmd_setD          );
}

void loop() {

  heater.update();
  heater.regulate();
  sendData();
  cmd.Process();
  timer.limitCPS(30);
  timer.counter++;
}

// ---------------------------------- RUN ----------------------------------- //
