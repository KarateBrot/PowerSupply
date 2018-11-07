#ifndef SENSOR_H
#define SENSOR_H

#include "Adafruit_INA219.h"                         // Current & voltage sensor


struct Sensor {};


struct Sensor_Load : public Sensor {

private:
  static Adafruit_INA219 _INA219;

public:
  Sensor_Load(void);

  void setPrecision(bool);

  double getCurrent(void) const;
  double getVoltage(void) const;
};


struct Sensor_Ambient : public Sensor {

public:
  void read(void);
};


#endif // SENSOR_H