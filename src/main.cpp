
#include <Vaporizer.h>

Vaporizer v;



// ================================== RUN =================================== //

void setup()
{

}

void loop()
{
  v.heater.update();
  v.heater.regulate();
}

// ---------------------------------- RUN ----------------------------------- //
