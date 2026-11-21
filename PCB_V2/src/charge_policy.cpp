#include "charge_policy.h"

namespace ChargePolicy {

uint8_t sanitiseLimit(int limit) {
    if (limit == 80 || limit == 90 || limit == 100) {
        return (uint8_t)limit;
    }
    return kLimitDefault;
}

Output step(Memory &mem, const Inputs &in) {
    Output out;
    if (!in.usb) {
        mem.held = false; // CE back to its default (charging on) for next time
        return out;       // UNPLUGGED
    }

    // Old boards: no CHG pin, no CE control. The charger always runs to its
    // own 4.2 V finish; call it complete once the gauge says full.
    if (!in.hasControl) {
        mem.held = false;
        out.state = (in.gaugeOk && in.soc >= 99.0f) ? State::COMPLETE : State::CHARGING;
        return out;
    }

    // A limit needs a working gauge: without one, leave the charger alone
    // (it still stops safely at 4.2 V).
    const uint8_t limit = sanitiseLimit(in.limit);
    const bool limiting = limit < 100 && in.gaugeOk;
    if (!limiting) {
        mem.held = false;
    } else if (mem.held) {
        if (in.soc < (float)limit - kResumeGap && in.volts < kResumeMaxVolts) {
            mem.held = false;
            // CE has only just gone low; CHG follows within a step
            out.state = State::CHARGING;
            return out;
        }
    } else if (in.soc >= (float)limit || in.volts >= kCapVolts) {
        mem.held = true;
    }

    if (mem.held) {
        out.disableCharging = true;
        out.state = State::COMPLETE;
        return out;
    }

    if (in.chgLow) {
        out.state = State::CHARGING;
    } else if (!in.gaugeOk || in.volts >= kFullVolts || in.soc >= kFullSoc) {
        // CHG goes high at the charger's own termination and stays high
        // through its top-up recharges, so a near-full cell is finished.
        // Without a gauge there is no telling a pause from a finish; the
        // charger says it isn't charging, so call it complete.
        out.state = State::COMPLETE;
    } else {
        out.state = State::PAUSED;
    }
    return out;
}

Led ledFor(State state, float soc) {
    switch (state) {
    case State::CHARGING:
        return soc < kOrangeFrom ? Led::RED : Led::ORANGE;
    case State::COMPLETE:
        return Led::GREEN;
    case State::PAUSED:
        return Led::ORANGE_PULSE;
    case State::UNPLUGGED:
    default:
        return Led::NONE;
    }
}

float displayPercent(float soc) {
    if (!(soc > 0.0f)) { // also catches NaN
        return 0.0f;
    }
    return soc > 100.0f ? 100.0f : soc;
}

} // namespace ChargePolicy
