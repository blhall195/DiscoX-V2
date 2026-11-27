#pragma once

#include "button_manager.h"
#include "fruity_menu.h"
#include "settings.h"
#include <Adafruit_SH110X.h>

/// What the loop should do once the menu closes
enum class MenuExit : uint8_t {
    NONE,         // still open
    BACK,         // back to the rangefinder screen
    UPDATE,       // firmware update (UF2 bootloader drive)
    POWER_OFF,    // "Power off" chosen
    TIMED_OUT,    // no press for the shutdown timeout
};

// The whole menu:
//   Exit
//   Shutdown: <n> ->   auto power-off after no button press
//   Laser off: <n> ->  laser beam timeout
//   Update ->          No / Yes: reboot into the UF2 drive
//   Power off
// Same widget (FruityMenu), button roles and timeout lists as the full
// firmware's MenuManager, so it behaves the same under the thumb.
class RfMenu {
  public:
    void begin(Adafruit_SH1107 &display, Settings &settings);
    /// Poll once per loop. Returns NONE while open.
    MenuExit update(ButtonManager &buttons);
    bool isActive() const { return _active; }
    void close() { _active = false; }

  private:
    void build();
    void finish(MenuExit e);

    FruityMenu _root, _shutdownSub, _laserSub, _updateSub;
    char _shutdownLabel[24];
    char _laserLabel[24];

    Adafruit_SH1107 *_display = nullptr;
    Settings *_settings = nullptr;
    bool _active = false;
    uint32_t _entryTime = 0;
    uint32_t _lastActivity = 0;
    MenuExit _exit = MenuExit::NONE;

    static RfMenu *s_instance;
    static void exitMenu(int);
    static void goToRoot(int);
    static void setShutdown(int seconds);
    static void setLaser(int seconds);
    static void startUpdate(int);
    static void powerOff(int);
};
