#include "feedback.h"
#include "sounds.h"

namespace Feedback {

void shotStart(DiscoManager &disco, bool keepLed) {
    if (!keepLed) {
        disco.setRed();
    }
    Sounds::shotStart();
}

void readingOk(DiscoManager &disco) {
    disco.setGreen();
    Sounds::readingOk();
}

void splayOk(DiscoManager &disco) {
    disco.setGreen();
    Sounds::splayOk();
}

void legComplete(DiscoManager &disco, LaserManager &laser, bool wibble) {
    disco.setWhite();
    Sounds::legComplete();
    disco.turnOff();
    if (wibble) {
        laser.wibble();
    }
    disco.setPurple();
}

void failed(DiscoManager &disco) {
    disco.blackout();
    for (int i = 0; i < 4; i++) {
        disco.setRed();
        delay(100);
        disco.blackout(); // dark gaps even over a charging colour
        delay(100);
    }
    Sounds::error();
    disco.turnOff(); // back to the base colour (dark when not charging)
}

} // namespace Feedback
