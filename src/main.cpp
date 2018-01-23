
// ---------------------------------- RUN ----------------------------------- //

#include <Vaporizer.h>




void setup()
{
  Vaporizer::init();
  GUI::showSplash(2, BASE);
}

void loop()
{
  Vaporizer::monitor();
  GUI::draw();
}

// -------------------------------------------------------------------------- //
