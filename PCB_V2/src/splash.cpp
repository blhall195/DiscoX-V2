#include "splash.h"

#include "version.h"
#include <math.h>
#include <string.h>

// Concepts were designed in a browser mock-up of the panel (same font, same
// 128x128 buffer); the maths here is a straight port of it. Stack use per
// frame is small — the one larger scratch array (cave ring vertices) is
// static, per the loop task's 4 KB stack limit (see CLAUDE.md).

namespace Splash {
namespace {

constexpr int16_t W = 128;
constexpr int16_t H = 128;
constexpr uint16_t WHITE = SH110X_WHITE;
constexpr uint16_t BLACK = SH110X_BLACK;
constexpr uint16_t INV = SH110X_INVERSE;
constexpr float PI_F = 3.14159265f;

constexpr const char *kNames[COUNT] = {"Classic",      "Laser Engrave", "Mirror Ball", "Compass Lock",
                                       "Drop & Shock", "Cave Flight",   "Disco Floor"};
constexpr uint16_t kDurations[COUNT] = {1450, 2700, 2700, 2500, 2100, 2600, 2600};

constexpr const char kWord[] = "DiscoX";
constexpr int16_t TITLE_X = 10; // "DiscoX" at size 3 is 108 px wide → centred at x=10
constexpr int16_t LETTER_W = 18;

// Deterministic per-frame "randomness" (sparks, glints, scramble chars).
uint32_t hash3(int32_t a, int32_t b = 0, int32_t c = 0) {
    uint32_t h = (uint32_t)a * 374761393u + (uint32_t)b * 668265263u + (uint32_t)c * 2147483647u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return h ^ (h >> 16);
}

float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

float easeOut(float v) {
    float u = 1.0f - clamp01(v);
    return 1.0f - u * u * u;
}

int16_t rnd(float v) { return (int16_t)lroundf(v); }

int16_t centreX(size_t len, uint8_t size = 1) { return (int16_t)((W - (int16_t)len * 6 * size) / 2); }

// Transparent-background text; n limits how many chars are drawn.
void text(Adafruit_GFX &g, int16_t x, int16_t y, const char *s, uint8_t size, uint16_t c = WHITE,
          size_t n = SIZE_MAX) {
    for (size_t k = 0; s[k] && k < n; k++) {
        g.drawChar(x + (int16_t)k * 6 * size, y, s[k], c, c, size);
    }
}

void hline(Adafruit_GFX &g, int16_t x, int16_t y, int16_t w, uint16_t c = WHITE) {
    if (w > 0) {
        g.drawFastHLine(x, y, w, c);
    }
}

void vline(Adafruit_GFX &g, int16_t x, int16_t y, int16_t h, uint16_t c = WHITE) {
    if (h > 0) { // GFX draws a negative height upwards — never wanted here
        g.drawFastVLine(x, y, h, c);
    }
}

void twinkle(Adafruit_GFX &g, int16_t x, int16_t y, int16_t s) {
    hline(g, x - s, y, 2 * s + 1);
    vline(g, x, y - s, 2 * s + 1);
}

// Bresenham with every other pixel skipped — reads as "further away".
void dottedLine(Adafruit_GFX &g, int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
    int16_t dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int16_t dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int16_t err = dx + dy;
    for (uint16_t n = 0;; n++) {
        if ((n & 1) == 0) {
            g.drawPixel(x0, y0, WHITE);
        }
        if (x0 == x1 && y0 == y1) {
            break;
        }
        int16_t e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void footer(Adafruit_GFX &g, const char *name, const char *ver, int16_t y1 = 108, int16_t y2 = 118,
            uint16_t c = WHITE) {
    if (name && name[0]) {
        text(g, centreX(strlen(name)), y1, name, 1, c);
    }
    text(g, centreX(strlen(ver)), y2, ver, 1, c);
}

// Footer typed out a character at a time — name, then version — with a
// blinking block cursor.
void typedFooter(Adafruit_GFX &g, const char *name, const char *ver, uint32_t t, uint16_t msPerChar,
                 int16_t y1 = 108, int16_t y2 = 118) {
    size_t nl = name ? strlen(name) : 0, vl = strlen(ver), n = t / msPerChar;
    size_t a = n < nl ? n : nl;
    size_t b = n > nl ? (n - nl < vl ? n - nl : vl) : 0;
    if (a) {
        text(g, centreX(nl), y1, name, 1, WHITE, a);
    }
    if (b) {
        text(g, centreX(vl), y2, ver, 1, WHITE, b);
    }
    if (n < nl + vl && ((t / 100) & 1) == 0) {
        bool onName = n < nl;
        int16_t x = onName ? centreX(nl) + (int16_t)a * 6 : centreX(vl) + (int16_t)b * 6;
        g.fillRect(x, onName ? y1 : y2, 5, 8, WHITE);
    }
}

// ── Device silhouette + laser beam (the original splash artwork) ──────
// The logo has concave TOP and BOTTOM edges (dipping inward toward the
// centre) with relatively straight vertical sides and rounded corners.
void drawDevice(Adafruit_GFX &g, int16_t cx = 46, int16_t cy = 64) {
    constexpr int16_t HW = 40;   // half-width
    constexpr int16_t HH = 20;   // half-height at the sides
    constexpr int16_t SCOOP = 6; // how far top/bottom edges dip inward

    // Two smooth dips with sharp cusps at the left edge, centre and right edge.
    auto bodyHH = [&](int16_t x) -> float {
        float t = (float)(x - (cx - HW)) / (float)(2 * HW); // 0..1
        return HH - SCOOP * fabsf(sinf(2.0f * 3.14159f * t));
    };

    // Trace top and bottom edges column by column (2-pass for thickness)
    for (int pass = 0; pass < 2; pass++) {
        int16_t prevTy = -1, prevBy = -1;
        for (int16_t x = cx - HW; x <= cx + HW; x++) {
            float hh = bodyHH(x) - (float)pass;
            int16_t ty = cy - (int16_t)hh;
            int16_t by = cy + (int16_t)hh;
            g.drawPixel(x, ty, WHITE);
            g.drawPixel(x, by, WHITE);
            if (x == cx - HW + pass || x == cx + HW - pass) {
                g.drawLine(x, ty, x, by, WHITE);
            }
            if (prevTy >= 0) {
                if (ty != prevTy) {
                    g.drawLine(x - 1, min(ty, prevTy), x, max(ty, prevTy), WHITE);
                }
                if (by != prevBy) {
                    g.drawLine(x - 1, min(by, prevBy), x, max(by, prevBy), WHITE);
                }
            }
            prevTy = ty;
            prevBy = by;
        }
    }

    // 4 buttons, then the disco square
    constexpr int16_t BTN = 8, GAP = 2;
    const int16_t bx0 = cx - HW + 8, by0 = cy - BTN / 2;
    for (int i = 0; i < 4; i++) {
        g.drawRoundRect(bx0 + i * (BTN + GAP), by0, BTN, BTN, 2, WHITE);
    }
    g.drawRoundRect(bx0 + 4 * (BTN + GAP) + 4, cy - 9, 22, 18, 4, WHITE);
}

void drawBeam(Adafruit_GFX &g, int16_t cx = 46, int16_t cy = 64) {
    const int16_t x0 = cx + 40 + 2, x1 = 112;
    g.drawLine(x0, cy, x1, cy, WHITE);
    g.drawLine(x0, cy - 1, x1, cy - 1, WHITE);
    // Starburst at the tip
    const int16_t sx = x1 + 2;
    for (int k = 0; k < 8; k++) {
        float a = k * 0.7854f;
        g.drawLine(sx, cy, sx + (int16_t)(cosf(a) * 8), cy + (int16_t)(sinf(a) * 8), WHITE);
    }
    g.fillCircle(sx, cy, 2, WHITE);
}

// ── Styles ────────────────────────────────────────────────────────────

void drawClassic(Adafruit_GFX &g, uint32_t t, const char *name, const char *ver) {
    // off(200) → on(300) → off(200) → on (final hold)
    bool on = t >= 200 && (t < 500 || t >= 700);
    text(g, TITLE_X, 8, kWord, 3);
    drawDevice(g);
    if (on) {
        drawBeam(g);
    }
    footer(g, name, ver);
}

void drawEngrave(Adafruit_GFX &g, uint32_t t, const char *name, const char *ver) {
    constexpr int16_t CX = 46, CY = 64, TY = 8;
    constexpr uint32_t E0 = 650, E1 = 1850; // engraving sweep
    const uint32_t fr = t / 50;

    // Device draws itself in left to right, then test-fires
    drawDevice(g, CX, CY);
    if (t < 400) {
        int16_t xr = CX - 40 + (int16_t)(81 * t / 400);
        g.fillRect(xr + 1, CY - 22, W, 45, BLACK);
    }
    if (t >= 400 && t < E0 && (fr & 1) == 0) {
        drawBeam(g, CX, CY);
    }

    if (t >= E0) {
        text(g, TITLE_X, TY, kWord, 3);
        if (t < E1) {
            // Hide what the beam hasn't reached; the beam rasters up and
            // down across the leading edge, throwing sparks
            float p = (float)(t - E0) / (float)(E1 - E0);
            int16_t xs = TITLE_X + (int16_t)(p * 108);
            g.fillRect(xs, TY - 1, W - xs, 26, BLACK);
            int16_t yd = TY + 11 + rnd(11.0f * sinf(t * 0.09f));
            g.drawLine(CX + 42, CY - 1, xs, yd, WHITE);
            g.drawLine(CX + 42, CY, xs, yd, WHITE);
            g.fillCircle(xs, yd, 1, WHITE);
            for (int k = 0; k < 6; k++) {
                uint32_t h = hash3(k, (int32_t)fr);
                g.drawPixel(xs + (int16_t)(h % 9) - 2, yd + (int16_t)((h >> 4) % 9) - 4, WHITE);
            }
        }
    }
    if (t >= E1 && t < E1 + 100) {
        g.fillRect(TITLE_X - 2, TY - 2, 112, 28, INV); // flash
    }
    if (t >= E1 + 100) {
        drawBeam(g, CX, CY);
    }
    if (t >= E1 + 150) {
        typedFooter(g, name, ver, t - (E1 + 150), 45);
    }
}

void drawMirrorBall(Adafruit_GFX &g, uint32_t t, const char *name, const char *ver) {
    constexpr int16_t R = 19, CX = 64, CYF = 33, TY = 72;
    // Drops from above the screen on a damped spring
    const int16_t cy = rnd(CYF - 58.0f * expf(-(float)t / 220.0f) * cosf((float)t / 105.0f));

    if (t > 500) { // twinkles (drawn first so the ball covers them)
        for (int k = 0; k < 6; k++) {
            uint32_t tk = t + k * 67, ph = tk % 400;
            uint32_t h = hash3(k, (int32_t)(tk / 400), 7);
            twinkle(g, 4 + h % 120, 2 + (h >> 8) % 62, (int16_t)((ph < 200 ? ph : 400 - ph) / 70));
        }
    }
    if (t > 600) { // dashed light rays, kept clear of the title
        for (int i = 0; i < 8; i++) {
            float a = i * PI_F / 4 + t * 0.0012f, ca = cosf(a), sa = sinf(a);
            for (int s = R + 5; s < R + 44; s += 7) {
                float y0 = cy + sa * s, y1 = cy + sa * (s + 3);
                if (y0 > 64 || y1 > 64) {
                    continue;
                }
                g.drawLine((int16_t)(CX + ca * s), (int16_t)y0, (int16_t)(CX + ca * (s + 3)), (int16_t)y1, WHITE);
            }
        }
    }

    vline(g, CX, 0, cy - R); // string
    g.fillCircle(CX, cy, R, BLACK);
    g.drawCircle(CX, cy, R, WHITE);
    for (int k = -2; k <= 2; k++) { // latitudes
        int16_t dy = rnd(k * R / 3.0f), hw = (int16_t)sqrtf((float)(R * R - dy * dy));
        hline(g, CX - hw, cy + dy, 2 * hw + 1);
    }
    // Meridians rotate with phi; the front-facing facets glint at random
    const float phi = t * 0.005f;
    for (int i = 0; i < 12; i++) {
        float th = phi + i * PI_F / 6;
        if (cosf(th) > 0) {
            for (int16_t dy = -R + 1; dy < R; dy++) {
                g.drawPixel(CX + rnd(sinf(th) * sqrtf((float)(R * R - dy * dy))), cy + dy, WHITE);
            }
        }
        float tm = th + PI_F / 12;
        if (cosf(tm) > 0.3f) {
            for (int b = -3; b < 3; b++) {
                if (hash3(i, b + 3, (int32_t)(t / 120)) % 7 != 0) {
                    continue;
                }
                int16_t dy = rnd((b + 0.5f) * R / 3.0f);
                int16_t x = CX + rnd(sinf(tm) * sqrtf((float)(R * R - dy * dy)));
                g.fillRect(x - 1, cy + dy - 1, 3, 3, WHITE);
            }
        }
    }

    // Letters light up one at a time, each arriving as an inverted flash
    for (int i = 0; i < 6; i++) {
        uint32_t at = 900 + i * 90;
        int16_t x = TITLE_X + i * LETTER_W;
        if (t < at) {
            continue;
        }
        if (t < at + 60) {
            g.fillRect(x - 1, TY - 2, 18, 26, WHITE);
            g.drawChar(x, TY, kWord[i], BLACK, BLACK, 3);
        } else {
            g.drawChar(x, TY, kWord[i], WHITE, WHITE, 3);
        }
    }
    if (t >= 1600) {
        footer(g, name, ver);
    }
}

constexpr char kJunk[] = "0123456789#%&@$?XZ+";
constexpr uint8_t kJunkLen = sizeof(kJunk) - 1;

// Text that shows random characters, locking each one in from the left.
void scrambleLine(Adafruit_GFX &g, const char *s, int16_t y, int32_t sinceStart, uint32_t fr) {
    size_t len = strlen(s);
    int16_t x0 = centreX(len);
    for (size_t k = 0; k < len; k++) {
        char ch = (sinceStart > 60 + (int32_t)k * 35) ? s[k] : kJunk[hash3((int32_t)k + 9, (int32_t)fr, y) % kJunkLen];
        g.drawChar(x0 + (int16_t)k * 6, y, ch, WHITE, WHITE, 1);
    }
}

void drawCompass(Adafruit_GFX &g, uint32_t t, const char *name, const char *ver) {
    constexpr int16_t CX = 64, CY = 34, R = 27, TY = 72;
    const uint32_t fr = t / 50;

    // Dial: ring, minor ticks, cardinal letters
    g.drawCircle(CX, CY, R, WHITE);
    for (int i = 0; i < 12; i++) {
        if (i % 3 == 0) {
            continue;
        }
        float a = i * PI_F / 6, s = sinf(a), c = cosf(a);
        g.drawLine((int16_t)(CX + s * (R - 4)), (int16_t)(CY - c * (R - 4)), (int16_t)(CX + s * (R - 1)),
                   (int16_t)(CY - c * (R - 1)), WHITE);
    }
    for (int i = 0; i < 4; i++) {
        float a = i * PI_F / 2;
        g.drawChar(CX + rnd(sinf(a) * (R - 7)) - 2, CY - rnd(cosf(a) * (R - 7)) - 3, "NESW"[i], WHITE, WHITE, 1);
    }

    // Needle: spins, swings, settles on north (damped oscillation)
    float h = 600.0f * expf(-(float)t / 350.0f) * cosf((float)t / 180.0f) * PI_F / 180.0f;
    float nx = sinf(h), ny = -cosf(h);
    constexpr float L = 15;
    g.fillTriangle((int16_t)(CX + nx * L), (int16_t)(CY + ny * L), (int16_t)(CX - ny * 3), (int16_t)(CY + nx * 3),
                   (int16_t)(CX + ny * 3), (int16_t)(CY - nx * 3), WHITE);
    g.drawLine((int16_t)(CX - nx * L), (int16_t)(CY - ny * L), (int16_t)(CX - ny * 3), (int16_t)(CY + nx * 3), WHITE);
    g.drawLine((int16_t)(CX - nx * L), (int16_t)(CY - ny * L), (int16_t)(CX + ny * 3), (int16_t)(CY - nx * 3), WHITE);
    g.fillCircle(CX, CY, 2, BLACK);
    g.drawCircle(CX, CY, 2, WHITE);
    if (t >= 1750 && t < 2050) {
        g.drawCircle(CX, CY, R + 2 + (int16_t)((t - 1750) / 30), WHITE); // lock "ping"
    }

    // Name decodes from noise, letter by letter
    for (int i = 0; i < 6; i++) {
        uint32_t start = 150 + i * 40, lock = 450 + i * 170;
        int16_t x = TITLE_X + i * LETTER_W;
        if (t < start) {
            continue;
        }
        if (t < lock) {
            g.drawChar(x, TY, kJunk[hash3(i, (int32_t)fr) % kJunkLen], WHITE, WHITE, 3);
        } else {
            g.drawChar(x, TY, kWord[i], WHITE, WHITE, 3);
            if (t - lock < 100) {
                hline(g, x, TY + 24, 16);
            }
        }
    }
    if (t >= 1500) {
        if (name && name[0]) {
            scrambleLine(g, name, 108, (int32_t)t - 1500, fr);
        }
        scrambleLine(g, ver, 118, (int32_t)t - 1550, fr);
    }
}

void drawDrop(Adafruit_GFX &g, uint32_t t, const char *name, const char *ver) {
    constexpr int16_t TY = 44;
    static constexpr uint16_t kStart[6] = {0, 110, 220, 330, 440, 650};
    auto y0Of = [](int i) { return i == 5 ? -60.0f : -30.0f; };     // X falls from higher...
    auto grOf = [](int i) { return i == 5 ? 0.0016f : 0.0009f; };   // ...and harder
    auto impactOf = [&](int i) { return kStart[i] + sqrtf(2.0f * (TY - y0Of(i)) / grOf(i)); };

    const float dX = (float)t - impactOf(5); // ms since the X landed
    int16_t ox = 0, oy = 0;                  // screen shake
    if (dX >= 0 && dX < 200) {
        float k = 1.0f - dX / 200.0f;
        uint32_t f = t / 50;
        ox = rnd(((f & 1) ? 3 : -3) * k);
        oy = rnd(((f & 2) ? 2 : -2) * k);
    }

    for (int i = 0; i < 6; i++) {
        int32_t tl = (int32_t)t - kStart[i];
        if (tl < 0) {
            continue;
        }
        // Integrate the fall + bounces from the start each frame (≤ ~420 steps)
        const float gr = grOf(i);
        float y = y0Of(i), v = 0;
        for (int32_t s = 0; s < tl; s += 5) {
            v += gr * 5;
            y += v * 5;
            if (y >= TY) {
                y = TY;
                v = fabsf(v) < 0.06f ? 0 : -v * 0.35f;
            }
        }
        int16_t x = TITLE_X + i * LETTER_W;
        g.drawChar(x + ox, rnd(y) + oy, kWord[i], WHITE, WHITE, 3);

        float d = (float)t - impactOf(i); // dust kicked up on first impact
        if (d >= 0 && d < 300) {
            for (int k = 0; k < 3; k++) {
                float dx = d * 0.05f * (k + 1), dy = d * 0.04f * (3 - k) - 0.000375f * d * d;
                g.drawPixel((int16_t)(x + 7 - dx) + ox, (int16_t)(TY + 22 - dy) + oy, WHITE);
                g.drawPixel((int16_t)(x + 8 + dx) + ox, (int16_t)(TY + 22 - dy) + oy, WHITE);
            }
        }
    }

    if (dX >= 0 && dX < 450) { // shockwave, inverted so it cuts through the letters
        int16_t r = (int16_t)(4 + dX * 0.21f);
        g.drawCircle(64 + ox, TY + 11 + oy, r, INV);
        g.drawCircle(64 + ox, TY + 11 + oy, r + 1, INV);
    }
    if (dX > 150) {
        int16_t hw = (int16_t)((dX - 150) * 0.3f);
        hw = hw > 54 ? 54 : hw;
        hline(g, 64 - hw + ox, TY + 29 + oy, 2 * hw);
    }
    if (dX > 250) {
        int16_t off = rnd(22.0f * (1.0f - easeOut((dX - 250) / 300.0f)));
        footer(g, name, ver, 108 + off, 118 + off);
    }
    if (dX >= 0 && dX < 50) {
        g.fillRect(0, 0, W, H, INV); // impact flash
    }
}

void drawCave(Adafruit_GFX &g, uint32_t t, const char *name, const char *ver) {
    constexpr int16_t TY = 44;
    constexpr uint32_t F0 = 1300, F1 = 1600, I0 = 1850, I1 = 2150;
    constexpr int N = 14;        // vertices per passage cross-section
    constexpr int MAX_RINGS = 10;
    static int16_t pts[MAX_RINGS][N][2]; // static: keep it off the 4 KB loop stack
    static float depth[MAX_RINGS];

    if (t < F1) {
        // Vanishing point wanders → the passage winds; camera accelerates
        const float vx = 64 + 8 * sinf(t * 0.0021f), vy = 60 + 6 * sinf(t * 0.0016f + 1);
        const float cam = 0.002f * t + 0.0000015f * (float)t * (float)t;
        int count = 0;
        for (int32_t j = (int32_t)ceilf(cam + 0.5f); j <= cam + 10 && count < MAX_RINGS; j++, count++) {
            float z = 0.55f * (j - cam), S = 40.0f / z;
            float mx = 0.35f * sinf(j * 0.9f), my = 0.15f * sinf(j * 1.7f);
            depth[count] = z;
            for (int m = 0; m < N; m++) {
                float th = m * 2 * PI_F / N;
                float rr = 1.0f + 0.5f * ((float)(hash3(j, m) % 256) / 255.0f - 0.5f);
                float sy = sinf(th) * rr;
                if (sy > 0.45f) {
                    sy = 0.45f + (sy - 0.45f) * 0.3f; // flattish floor
                }
                pts[count][m][0] = (int16_t)(vx + (mx + cosf(th) * rr * 1.3f) * S);
                pts[count][m][1] = (int16_t)(vy + (my + sy * 0.9f) * S);
            }
        }
        for (int r = 0; r < count; r++) {
            bool far = depth[r] > 3;
            for (int m = 0; m < N; m++) {
                const int16_t *a = pts[r][m], *b = pts[r][(m + 1) % N];
                if (far) {
                    dottedLine(g, a[0], a[1], b[0], b[1]);
                } else {
                    g.drawLine(a[0], a[1], b[0], b[1], WHITE);
                }
            }
            if (r + 1 < count) {
                static constexpr uint8_t kRails[] = {0, 4, 7, 10};
                for (uint8_t m : kRails) {
                    dottedLine(g, pts[r][m][0], pts[r][m][1], pts[r + 1][m][0], pts[r + 1][m][1]);
                }
            }
        }
        g.fillCircle((int16_t)vx, (int16_t)vy, 1, WHITE);
        if (t >= F0) { // light at the end grows to fill the screen
            float p = (float)(t - F0) / (float)(F1 - F0);
            g.fillCircle((int16_t)vx, (int16_t)vy, (int16_t)(p * p * 95), WHITE);
        }
    } else {
        // White screen, name cut out in black; then an iris opens and the
        // inverse-drawn text flips to white-on-black inside it
        g.fillRect(0, 0, W, H, WHITE);
        if (t >= I0) {
            g.fillCircle(64, 55, (int16_t)(easeOut((float)(t - I0) / (float)(I1 - I0)) * 95), BLACK);
        }
        text(g, TITLE_X, TY, kWord, 3, INV);
        if (t >= 2050) {
            footer(g, name, ver, 108, 118, INV);
        }
    }
}

void drawFloor(Adafruit_GFX &g, uint32_t t, const char *name, const char *ver) {
    constexpr int16_t TY = 6;
    const int16_t HY = rnd(128 - 68 * easeOut(t / 450.0f)); // horizon rises into place
    const uint32_t beat = t / 250;

    if (HY < 127) {
        const float scroll = t * 0.004f, base = floorf(scroll), frac = scroll - base;
        const float span = (float)(128 - HY);
        auto yAt = [&](float z) { return HY + span / z; };
        auto xAt = [&](int m, float y) { return 64 + m * 30 * (y - HY) / span; };

        // Lit tiles, re-rolled every beat, scrolling with the floor
        for (int k = 0; k < 10; k++) {
            float zn = k + 1 - frac;
            if (zn < 0.6f) {
                continue;
            }
            long yb = lroundf(yAt(zn)), yt = lroundf(yAt(zn + 1));
            yb = yb > 127 ? 127 : yb;
            for (int m = -5; m < 5; m++) {
                if (hash3(k + (int32_t)base, m + 9, (int32_t)beat) % 4 != 0) {
                    continue;
                }
                for (long y = yt + 1; y < yb; y++) {
                    int16_t xl = (int16_t)ceilf(xAt(m, y)) + 1, xr = (int16_t)floorf(xAt(m + 1, y)) - 1;
                    hline(g, xl, (int16_t)y, xr - xl + 1);
                }
            }
        }
        for (int k = 0; k < 12; k++) { // grid
            long y = lroundf(yAt(k + 1 - frac));
            if (y < 128) {
                hline(g, 0, (int16_t)y, W);
            }
        }
        for (int m = -5; m <= 5; m++) {
            g.drawLine((int16_t)xAt(m, 127), 127, (int16_t)(64 + m * 0.5f), HY, WHITE);
        }
    }
    g.fillRect(0, 0, W, HY, BLACK);
    hline(g, 0, HY, W);

    for (int k = 0; k < 5; k++) { // stars, kept off the text rows
        uint32_t tk = t + k * 90, ph = tk % 450;
        uint32_t h = hash3(k, (int32_t)(tk / 450), 3);
        int16_t y = 2 + (int16_t)((h >> 8) % (uint32_t)max(1, HY - 4));
        if (y > 30 && y < 56) {
            continue;
        }
        twinkle(g, 3 + h % 122, y, (int16_t)((ph < 225 ? ph : 450 - ph) / 80));
    }

    if (t >= 500) { // lands on the first beat, then alternate letters hop
        float ph = (t % 250) / 250.0f;
        for (int i = 0; i < 6; i++) {
            int16_t hop = (t < 2100 && ((i + beat) & 1)) ? rnd(4 * sinf(PI_F * ph)) : 0;
            g.drawChar(TITLE_X + i * LETTER_W, TY + 4 - hop, kWord[i], WHITE, WHITE, 3);
        }
        if (t < 550) {
            g.fillRect(TITLE_X - 2, TY + 1, 112, 28, INV);
        }
    }
    if (t >= 900) {
        typedFooter(g, name, ver, t - 900, 40, 36, 46);
    }
}

} // namespace

const char *name(uint8_t style) { return kNames[style < COUNT ? style : CLASSIC]; }

uint16_t durationMs(uint8_t style) { return kDurations[style < COUNT ? style : CLASSIC]; }

void draw(Adafruit_GFX &g, uint8_t style, uint32_t t, const char *nameSuffix, const char *version) {
    switch (style) {
    case ENGRAVE:
        drawEngrave(g, t, nameSuffix, version);
        break;
    case MIRROR_BALL:
        drawMirrorBall(g, t, nameSuffix, version);
        break;
    case COMPASS:
        drawCompass(g, t, nameSuffix, version);
        break;
    case DROP:
        drawDrop(g, t, nameSuffix, version);
        break;
    case CAVE:
        drawCave(g, t, nameSuffix, version);
        break;
    case FLOOR:
        drawFloor(g, t, nameSuffix, version);
        break;
    default:
        drawClassic(g, t, nameSuffix, version);
        break;
    }
}

void play(Adafruit_SH1107 &display, uint8_t style, const char *nameSuffix) {
    if (style >= COUNT) {
        style = CLASSIC;
    }
    const uint32_t dur = durationMs(style);
    const uint32_t start = millis();
    // No frame pacing needed: display() is a ~2 KB I2C transfer that takes
    // ~45 ms at 400 kHz, which sets the rate. The last pass draws t = dur
    // exactly, so the final frame is always the one left on screen.
    for (;;) {
        uint32_t t = millis() - start;
        bool last = t >= dur;
        display.clearDisplay();
        draw(display, style, last ? dur : t, nameSuffix, FIRMWARE_VERSION);
        display.display();
        if (last) {
            break;
        }
    }
}

} // namespace Splash
