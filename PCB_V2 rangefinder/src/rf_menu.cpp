#include "rf_menu.h"

#include <stdio.h>

RfMenu *RfMenu::s_instance = nullptr;

void RfMenu::begin(Adafruit_SH1107 &display, Settings &settings) {
    _display = &display;
    _settings = &settings;
    _active = true;
    _exit = MenuExit::NONE;
    _entryTime = _lastActivity = millis();
    s_instance = this;
    Serial.println(F("Menu open"));
    build();
    _root.show();
}

MenuExit RfMenu::update(ButtonManager &buttons) {
    // Ignore input briefly so the MENU press that opened the menu can't
    // also select "Exit" (same grace period as the full firmware)
    if (millis() - _entryTime < 300) {
        buttons.wasPressed(Button::FIRE);
        buttons.wasPressed(Button::UP_DISCO);
        buttons.wasPressed(Button::DOWN);
        buttons.wasPressed(Button::MENU);
        return MenuExit::NONE;
    }

    bool interacted = false;
    if (buttons.wasPressed(Button::UP_DISCO)) {
        _root.scroll(-1);
        interacted = true;
    }
    if (buttons.wasPressed(Button::DOWN)) {
        _root.scroll(1);
        interacted = true;
    }
    if (buttons.wasPressed(Button::MENU) || buttons.wasPressed(Button::FIRE)) {
        _root.click();
        interacted = true;
    }
    if (!_active) {
        return _exit; // a callback closed the menu
    }
    if (interacted) {
        _lastActivity = millis();
        _root.show();
    }
    if (millis() - _lastActivity > _settings->autoShutdownTimeout * 1000UL) {
        Serial.println(F("Menu: inactivity timeout"));
        _active = false;
        return MenuExit::TIMED_OUT;
    }
    return MenuExit::NONE;
}

void RfMenu::finish(MenuExit e) {
    _active = false;
    _exit = e;
}

void RfMenu::build() {
    const uint32_t sd = _settings->autoShutdownTimeout;
    if (sd >= 7200 && sd % 3600 == 0) {
        snprintf(_shutdownLabel, sizeof(_shutdownLabel), "Shutdown: %lu hr", (unsigned long)(sd / 3600));
    } else {
        snprintf(_shutdownLabel, sizeof(_shutdownLabel), "Shutdown: %lu min", (unsigned long)(sd / 60));
    }
    const uint32_t lt = _settings->laserTimeout;
    if (lt >= 61) {
        snprintf(_laserLabel, sizeof(_laserLabel), "Laser off: %lu min", (unsigned long)(lt / 60));
    } else {
        snprintf(_laserLabel, sizeof(_laserLabel), "Laser off: %lu sec", (unsigned long)lt);
    }

    _root.init(*_display, "Menu");
    _shutdownSub.init(*_display, _shutdownLabel);
    _laserSub.init(*_display, _laserLabel);
    _updateSub.init(*_display, "Update firmware?");

    _root.addAction("Exit", exitMenu);
    _root.addSubmenu(_shutdownLabel, &_shutdownSub);
    _root.addSubmenu(_laserLabel, &_laserSub);
    _root.addSubmenu("Update", &_updateSub);
    _root.addAction("Power off", powerOff);

    // Same choices as the full firmware's Settings submenus
    _shutdownSub.addAction("5 min", setShutdown, 300);
    _shutdownSub.addAction("10 min", setShutdown, 600);
    _shutdownSub.addAction("15 min", setShutdown, 900);
    _shutdownSub.addAction("30 min", setShutdown, 1800);
    _shutdownSub.addAction("60 min", setShutdown, 3600);
    _shutdownSub.addAction("2 hr", setShutdown, 7200);
    _shutdownSub.addAction("<- Back", goToRoot);

    _laserSub.addAction("30 sec", setLaser, 30);
    _laserSub.addAction("60 sec", setLaser, 60);
    _laserSub.addAction("2 min", setLaser, 120);
    _laserSub.addAction("5 min", setLaser, 300);
    _laserSub.addAction("15 min", setLaser, 900);
    _laserSub.addAction("30 min", setLaser, 1800);
    _laserSub.addAction("<- Back", goToRoot);

    _updateSub.addAction("No", goToRoot);
    _updateSub.addAction("Yes", startUpdate);
    _updateSub.addAction("<- Back", goToRoot);
}

// ── Callbacks ───────────────────────────────────────────────────────
// Picking a value rebuilds the menu (labels change) and lands back on the
// root, as the full firmware's buildMenu() does.

void RfMenu::exitMenu(int) {
    if (s_instance) {
        s_instance->finish(MenuExit::BACK);
    }
}

void RfMenu::goToRoot(int) {
    if (s_instance) {
        s_instance->_root.closeSub();
    }
}

void RfMenu::setShutdown(int seconds) {
    if (!s_instance) {
        return;
    }
    s_instance->_settings->autoShutdownTimeout = (uint32_t)seconds;
    SettingsStore::save(*s_instance->_settings);
    Serial.print(F("Menu: shutdown timeout = "));
    Serial.println(seconds);
    s_instance->build();
}

void RfMenu::setLaser(int seconds) {
    if (!s_instance) {
        return;
    }
    s_instance->_settings->laserTimeout = (uint32_t)seconds;
    SettingsStore::save(*s_instance->_settings);
    Serial.print(F("Menu: laser timeout = "));
    Serial.println(seconds);
    s_instance->build();
}

void RfMenu::startUpdate(int) {
    if (s_instance) {
        s_instance->finish(MenuExit::UPDATE);
    }
}

void RfMenu::powerOff(int) {
    if (s_instance) {
        s_instance->finish(MenuExit::POWER_OFF);
    }
}
