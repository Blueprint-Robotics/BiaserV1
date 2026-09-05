/*
 * STM32 Blackpill line follower with selectable bias.
 *
 * A 14-channel IR sensor array is read through a 4-bit multiplexer and fed
 * into a PID controller. Three modes are selectable from a small button-driven
 * menu on an SSD1306 OLED:
 *
 *   LEFT BIAS  - favors the left half of the sensor array, for tracks that
 *                turn sharply left.
 *   STRAIGHT   - uses the full array with no side preference.
 *   RIGHT BIAS - mirror of LEFT BIAS.
 *
 * Hardware: STM32 "Blackpill" (STM32F4x1), analog IR array via mux, two
 * H-bridge driven DC motors, SSD1306 128x64 I2C OLED, three push buttons.
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>

// ---- Pin assignments ----
#define PIN_MUX_S0   PA3
#define PIN_MUX_S1   PA5
#define PIN_MUX_S2   PA15
#define PIN_MUX_S3   PB0
#define PIN_MUX_SIG  PA4

#define PIN_BTN_UP      PB10
#define PIN_BTN_DOWN    PB1
#define PIN_BTN_SELECT  PB4

#define PIN_I2C_SCL PB8
#define PIN_I2C_SDA PB9

#define PIN_L_LPWM   PA10
#define PIN_L_RPWM   PA11
#define PIN_R_RPWM   PA9
#define PIN_R_LPWM   PA8

#define ADDR_OLED    0x3C

#define screenWidth  128
#define screenHeight 64

// ---- Sensor / motor constants ----
#define NUM_SENSORS    14
#define THRESHOLD      3000   // analogRead value above which a sensor counts as "on the line"
#define MUX_SETTLE_US  5      // mux settling time before each analogRead

#define BASE_SPEED     200    // constant drive speed; steering comes entirely from the PID correction
#define DEBOUNCE_MS    100

// ---- PID gains ----
const float KP = 100.0f;
const float KI = 0.0f;
const float KD = 60.0f;

float lastError    = 0.0f;
float currentError = 0.0f;   // last computed error, kept around for the OLED readout
float integral      = 0.0f;
float direction     = 15.0f;   // last-known turn direction; used to keep steering the same way while the line is lost

unsigned long noLinesTimer    = 0;
unsigned long noLinesDebounce = 90;   // ms with no line seen before we commit to the recovery direction

// Physical offset of each sensor from the array's center, in arbitrary
// units. Used to turn a set of "on" sensors into a single position error.
static const float SENSOR_POSITION[NUM_SENSORS] = {
    -6.5f, -5.5f, -4.5f, -3.5f,
    -2.5f, -1.5f, -0.5f,  0.5f,
     1.5f,  2.5f,  3.5f,  4.5f,
     5.5f,  6.5f
};

// Per-sensor weights applied on top of SENSOR_POSITION when a bias mode is
// active. Left bias weights the leftmost sensors heavily so the controller
// reacts strongly to the line drifting further left; right bias is the
// mirror image. Straight mode ignores these entirely.
static const float SENSOR_WEIGHTS_LEFT[NUM_SENSORS] = {
    5.0f, 5.0f, 5.0f, 8.0f,
    9.0f, 10.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f
};

static const float SENSOR_WEIGHTS_RIGHT[NUM_SENSORS] = {
    1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f,
    10.0f, 9.0f, 8.0f, 5.0f,
    5.0f, 5.0f
};

int sensorVal[NUM_SENSORS] = {0};

// ---- Bias selection (nav menu) ----
enum BiasMode { BIAS_LEFT = 0, BIAS_STRAIGHT = 1, BIAS_RIGHT = 2 };
const char* biasNames[3] = { "LEFT BIAS", "STRAIGHT", "RIGHT BIAS" };

int  navCursor = 1;   // defaults to STRAIGHT
bool running   = false;

bool btnUpWasDown   = false;
bool btnDownWasDown = false;
bool btnSelWasDown  = false;

Adafruit_SSD1306 oled(screenWidth, screenHeight, &Wire, -1);

// ============================================================
// IR sensor array
// ============================================================

void sensorsInit() {
    pinMode(PIN_MUX_S0, OUTPUT);
    pinMode(PIN_MUX_S1, OUTPUT);
    pinMode(PIN_MUX_S2, OUTPUT);
    pinMode(PIN_MUX_S3, OUTPUT);
    pinMode(PIN_MUX_SIG, INPUT);
}

// Selects one of the 14 mux channels via its 4-bit address.
static void muxSelect(uint8_t ch) {
    digitalWrite(PIN_MUX_S0, (ch >> 0) & 1);
    digitalWrite(PIN_MUX_S1, (ch >> 1) & 1);
    digitalWrite(PIN_MUX_S2, (ch >> 2) & 1);
    digitalWrite(PIN_MUX_S3, (ch >> 3) & 1);
    delayMicroseconds(MUX_SETTLE_US);
}

// Reads all 14 sensors through the mux and thresholds each one to a
// simple on/off line-detection value.
void readSensors(int* values) {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        muxSelect(i);
        int reading = analogRead(PIN_MUX_SIG);
        values[i] = reading > THRESHOLD ? 1 : 0;
    }
}

// ============================================================
// Motor driver
// ============================================================

// Drives one motor on a two-pin (LPWM/RPWM style) H-bridge input.
void driveMotor(int pwmFwd, int pwmRev, int speed) {
    speed = constrain(speed, -220, 220);
    if (speed > 0) {
        analogWrite(pwmFwd, speed);
        analogWrite(pwmRev, 0);
    } else if (speed < 0) {
        analogWrite(pwmFwd, 0);
        analogWrite(pwmRev, abs(speed));
    } else {
        analogWrite(pwmFwd, 0);
        analogWrite(pwmRev, 0);
    }
}

void stopMotors() {
    driveMotor(PIN_L_LPWM, PIN_L_RPWM, 0);
    driveMotor(PIN_R_RPWM, PIN_R_LPWM, 0);
}

// ============================================================
// Debounced button readers
// ============================================================
// Each returns true once per physical press (rising edge), with a simple
// blocking debounce delay to ride out contact bounce.

bool btnUpPressed() {
    bool cur = (digitalRead(PIN_BTN_UP) == LOW);
    if (cur && !btnUpWasDown) { delay(DEBOUNCE_MS); btnUpWasDown = true; return true; }
    if (!cur) btnUpWasDown = false;
    return false;
}

bool btnDownPressed() {
    bool cur = (digitalRead(PIN_BTN_DOWN) == LOW);
    if (cur && !btnDownWasDown) { delay(DEBOUNCE_MS); btnDownWasDown = true; return true; }
    if (!cur) btnDownWasDown = false;
    return false;
}

bool btnSelectPressed() {
    bool cur = (digitalRead(PIN_BTN_SELECT) == LOW);
    if (cur && !btnSelWasDown) { delay(DEBOUNCE_MS); btnSelWasDown = true; return true; }
    if (!cur) btnSelWasDown = false;
    return false;
}

// ============================================================
// Nav menu - choose one of the three bias modes before starting
// ============================================================

void drawNavMain() {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);

    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print(F("Select bias, SEL=Start"));

    oled.setTextSize(2);
    int chars = strlen(biasNames[navCursor]);
    int xTxt  = (screenWidth - chars * 12) / 2;
    oled.setCursor(max(0, xTxt), 26);
    oled.print(biasNames[navCursor]);

    oled.setTextSize(1);
    oled.setCursor(0, 56);
    oled.print(F("UP/DN:cycle SEL:start"));
    oled.display();
}

void handleNav() {
    if (btnDownPressed()) { navCursor = (navCursor + 1) % 3; drawNavMain(); }
    if (btnUpPressed())   { navCursor = (navCursor - 1 + 3) % 3; drawNavMain(); }
    if (btnSelectPressed()) {
        lastError = 0.0f;
        integral  = 0.0f;
        direction = 15.0f;
        noLinesTimer = 0;
        oled.clearDisplay();
        oled.display();
        running = true;
    }
}

// ============================================================
// PID line-following with left/straight/right bias
//
//   Straight:   uses all 14 sensors, unweighted.
//   Left bias:  only sensors 0-6 (left of center) are considered, with
//               SENSOR_WEIGHTS_LEFT applied once enough of them are active.
//   Right bias: mirror of left bias over sensors 7-13.
//   No line:    once the line has been missing for noLinesDebounce ms,
//               steer using the last-known direction until it reappears.
// ============================================================
float computePID(int* values, BiasMode mode) {
    // Scan the whole array first - this decides whether the line is
    // visible at all, independent of whichever subset the current bias
    // mode restricts itself to below.
    int activeFull = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (values[i]) activeFull++;
    }

    if (activeFull == NUM_SENSORS) {
        // Every sensor sees the line at once - treat this as a stop/finish marker.
        stopMotors();
        while (1) {}
    }

    // Left = strictly left-of-center (positions -6.5..-0.5), right =
    // strictly right-of-center (positions +0.5..+6.5), with no overlap
    // at the middle sensor.
    int startIdx = 0, endIdx = NUM_SENSORS;
    if (mode == BIAS_LEFT) {
        startIdx = 0;
        endIdx   = 7;
    } else if (mode == BIAS_RIGHT) {
        startIdx = 7;
        endIdx   = NUM_SENSORS;
    }

    // Count how many sensors in the current bias's subset are active,
    // independent of weighting, to decide below whether the weight array
    // should be applied yet.
    int subsetActiveCount = 0;
    for (int i = startIdx; i < endIdx; i++) {
        if (values[i]) subsetActiveCount++;
    }

    // Straight never applies a weight array. Left/right bias only apply
    // theirs once more than 2 sensors on their own side are active, so a
    // single stray reading doesn't yank the weighting on.
    bool applyBiasWeight = (mode != BIAS_STRAIGHT) && (subsetActiveCount > 2);
    const float* weights = applyBiasWeight
                          ? (mode == BIAS_LEFT ? SENSOR_WEIGHTS_LEFT : SENSOR_WEIGHTS_RIGHT)
                          : nullptr;

    float weightedSum   = 0.0f;
    int   activeSensors = 0;
    for (int i = startIdx; i < endIdx; i++) {
        if (values[i]) {
            weightedSum += SENSOR_POSITION[i] * (weights ? weights[i] : 1.0f);
            activeSensors++;
        }
    }

    // The bias subset saw nothing, but the line is still visible
    // somewhere on the full array (it drifted outside the subset window)
    // - fall back to the full array instead of declaring the line lost.
    if (activeSensors == 0 && activeFull > 0) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (values[i]) {
                weightedSum += SENSOR_POSITION[i] * (weights ? weights[i] : 1.0f);
                activeSensors++;
            }
        }
    }

    float error;
    if (activeSensors == 0) {
        if (noLinesTimer == 0) {
            noLinesTimer = millis();
            error = 0.0f;
        } else if (millis() - noLinesTimer >= noLinesDebounce) {
            error = direction;
        } else {
            error = 0.0f;
        }
    } else {
        noLinesTimer = 0;
        error = -(weightedSum / (float)activeSensors);
    }

    // Recovery direction once the line is lost: straight mode tracks the
    // sign of the last error dynamically, while left/right bias always
    // recover toward their own side.
    if (mode == BIAS_STRAIGHT) {
        if (error > 0.0f)      direction =  15.0f;
        else if (error < 0.0f) direction = -15.0f;
    } else if (mode == BIAS_LEFT) {
        direction = -15.0f;
    } else if (mode == BIAS_RIGHT) {
        direction =  15.0f;
    }

    integral += error;
    integral = constrain(integral, -100.0f, 100.0f);

    float derivative = error - lastError;
    float output = (KP * error) + (KI * integral) + (KD * derivative);
    lastError    = error;
    currentError = error;

    return output;
}

// ============================================================
// Running-mode display - live PID readout
// ============================================================
void updateDisplay(BiasMode mode, float error, float correction, int leftSpeed, int rightSpeed) {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);

    oled.setCursor(0, 0);
    oled.print(biasNames[mode]);

    oled.setCursor(0, 16);
    oled.print(F("Error: "));
    oled.print(error, 2);

    oled.setCursor(0, 28);
    oled.print(F("Corr : "));
    oled.print(correction, 2);

    oled.setCursor(0, 40);
    oled.print(F("L:"));
    oled.print(leftSpeed);
    oled.print(F("  R:"));
    oled.print(rightSpeed);

    oled.setCursor(0, 56);
    oled.print(F("SEL: stop"));
    oled.display();
}

// ============================================================
// Setup
// ============================================================
void setup() {
    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SCL);
    Wire.begin();
    Wire.setClock(400000);
    delay(100);

    analogReadResolution(12);

    sensorsInit();

    pinMode(PIN_L_LPWM, OUTPUT); pinMode(PIN_L_RPWM, OUTPUT);
    pinMode(PIN_R_RPWM, OUTPUT); pinMode(PIN_R_LPWM, OUTPUT);

    pinMode(PIN_BTN_UP,     INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN,   INPUT_PULLUP);
    pinMode(PIN_BTN_SELECT, INPUT_PULLUP);

    oled.begin(SSD1306_SWITCHCAPVCC, ADDR_OLED);
    oled.clearDisplay();
    oled.display();

    drawNavMain();
}

// ============================================================
// Main loop
// ============================================================
void loop() {
    if (!running) {
        handleNav();
        return;
    }

    // Emergency stop: holding SELECT while running drops back to the menu.
    if (digitalRead(PIN_BTN_SELECT) == LOW) {
        stopMotors();
        running = false;
        drawNavMain();
        delay(300);
        return;
    }

    readSensors(sensorVal);

    float correction = computePID(sensorVal, (BiasMode)navCursor);
    int   leftSpeed   = (int)(BASE_SPEED + correction);
    int   rightSpeed  = (int)(BASE_SPEED - correction);

    updateDisplay((BiasMode)navCursor, currentError, correction, leftSpeed, rightSpeed);

    driveMotor(PIN_L_LPWM, PIN_L_RPWM, leftSpeed);
    driveMotor(PIN_R_RPWM, PIN_R_LPWM, rightSpeed);
}
