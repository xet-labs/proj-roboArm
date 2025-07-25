#include "lgc.h"
#include "../lib/lib.h"
#include <ESP32Servo.h>
#include <AccelStepper.h>

namespace lgc
{
    // Data Type
    struct __attribute__((packed)) Joypad
    {
        uint16_t btns;
        int16_t axisLX;
        int16_t axisLY;
        int16_t axisRX;
        int16_t axisRY;
        uint8_t triggerLT;
        uint8_t triggerRT;
    };

    enum Button : uint16_t
    {
        BTN_A = 1 << 0,
        BTN_B = 1 << 1,
        BTN_X = 1 << 2,
        BTN_Y = 1 << 3,
        BTN_LB = 1 << 4,
        BTN_RB = 1 << 5,
        BTN_BACK = 1 << 6,
        BTN_START = 1 << 7,
        BTN_LS = 1 << 8,
        BTN_RS = 1 << 9,
        DPAD_UP = 1 << 10,
        DPAD_DOWN = 1 << 11,
        DPAD_LEFT = 1 << 12,
        DPAD_RIGHT = 1 << 13
    };

    const int moveInterval = 15;

    // --CoreAction--
    // static Joypad lastInput = {};
    static unsigned long lastPrint = 0;

    // === Default Speeds ===
    constexpr int servoStepMin = 1;
    constexpr int servoStepDef = 1;
    constexpr int servoStepMax = 6;
    constexpr int servoRangeMin = 0;
    constexpr int servoRangeMax = 180;
    int servoStep = servoStepDef;
    int servo2Pos = 90, servo0Pos = 90, servo1Pos = 90;
    int dcSpeed;
    constexpr int dcSpeedMin = 100;
    constexpr int dcSpeedDef = 160;
    constexpr int dCSpeedMax = 255;
    constexpr int stepperStepMin = 1;
    constexpr int stepperStepDef = 1;
    constexpr int stepperStepMax = 6;
    int stepperStep = stepperStepDef;
    // int stepper0Pos = stepper[0].currentPosition();
    int stepper0Pos = 0;
    bool dpadUp, dpadDown, dpadLeft, dpadRight;

    void servoForward(Servo &servo, int &currentPos, int step = 1, int delayMs = 15, int rangeMax = 180, int rangeMin = 0)
    {
        static unsigned long lastServoForward = 0;
        unsigned long now = millis();

        if (now - lastServoForward >= delayMs)
        {
            lastServoForward = now;

            currentPos = min(rangeMax, currentPos + step);
            servo.write(currentPos);
        }
    }
    void servoBackward(Servo &servo, int &currentPos, int step = 1, int delayMs = 15, int rangeMin = 0, int rangeMax = 180)
    {
        static unsigned long lastUpdated = 0;
        unsigned long now = millis();

        if (now - lastUpdated >= delayMs)
        {
            lastUpdated = now;

            currentPos = max(rangeMin, currentPos - step);
            servo.write(currentPos);
        }
    }
    void stepperForward(AccelStepper &stepper, int &currentPos, int step = 1, int delayMs = 14, int rangeMax = 200, int rangeMin = -200)
    {
        static unsigned long lastServoBackward = 0;
        unsigned long now = millis();

        if (now - lastServoBackward >= delayMs)
        {
            lastServoBackward = now;

            currentPos = min(rangeMax, currentPos + step);
            stepper.moveTo(currentPos);
            stepper.run();
        }
    }
    void stepperBackward(AccelStepper &stepper, int &currentPos, int step = 1, int delayMs = 14, int rangeMin = -200, int rangeMax = 200)
    {
        static unsigned long lastUpdated = 0;
        unsigned long now = millis();

        if (now - lastUpdated >= delayMs)
        {
            lastUpdated = now;

            currentPos = max(rangeMin, currentPos - step);
            stepper.moveTo(currentPos);
            stepper.run();
        }
    }

    void coreAction(const Joypad &jp)
    {
        // Servos speed
        servoStep = jp.triggerRT > 0 ? map(jp.triggerRT, 0, 255, servoStepMin, servoStepMax) : servoStepDef;
        // Dc Motor Speed
        motor[0].setSpeed(jp.triggerLT > 0 ? jp.triggerLT : dcSpeedDef);

        stepper[0].setMaxSpeed(500);
        stepper[0].setAcceleration(200);

        // === DPAD Controls ===
        dpadUp = jp.btns & DPAD_UP;
        dpadDown = jp.btns & DPAD_DOWN;
        dpadLeft = jp.btns & DPAD_LEFT;
        dpadRight = jp.btns & DPAD_RIGHT;

        // === Stepper Movement (Still using axisLX) ===
        // int stepTarget = map(jp.axisLX, -32768, 32767, -200, 200);
        // stepper[0].moveTo(stepTarget);
        // stepper[0].run();

        // === DC Motor Control (RB = CW, LB = CCW) ===
        switch (jp.btns & (BTN_RB | BTN_LB))
        {
        case BTN_RB:
            motor[0].forward();
            break;

        case BTN_LB:
            motor[0].backward();
            break;

        case (BTN_RB | BTN_LB): // both pressed
            motor[0].stop();    // or do something else
            break;

        default:
            motor[0].stop();
            break;
        }

        // === Optional: Debug Info Every Second ===
        // if (millis() - lastPrint > 1000)
        // {
        // lastPrint = millis();
        Serial.printf("[JOY] soldr: %d, Elbow: %d, Head: %d | Stepper: %d | servoStep: %d\n",
                      servo0Pos, servo1Pos, servo2Pos, stepper0Pos,
                      (jp.btns & BTN_RB) ? "CW" : (jp.btns & BTN_LB) ? "CCW"
                                                                     : "STOP",
                      servoStep);
        // }
    }

    void coreActionAsync(const Joypad &jp)
    {
        // Stepper movement

        if (dpadLeft)
            stepperBackward(stepper[0], stepper0Pos, stepperStep, 14);
        if (dpadRight)
            stepperForward(stepper[0], stepper0Pos, stepperStep, 14);

        // soldr control (A + UP/DOWN)
        if (jp.btns & BTN_A)
        {
            if (dpadUp)
                servoForward(servo[0], servo0Pos, servoStep, 18);
            // servo0Pos = std::min(280, servo0Pos + servoStep);
            if (dpadDown)
                servoBackward(servo[0], servo0Pos, servoStep, 18);
            // servo0Pos = std::max(20, servo0Pos - servoStep);
        }

        // Elbow control (B + UP/DOWN)
        if (jp.btns & BTN_B)
        {
            if (dpadUp)
                servoBackward(servo[1], servo1Pos, servoStep, 18);
            // servo1Pos = std::max(0, servo1Pos - servoStep);
            if (dpadDown)
                servoForward(servo[1], servo1Pos, servoStep, 18);
            // servo1Pos = std::min(180, servo1Pos + servoStep);
        }

        // Head control (X + UP/DOWN)
        if (jp.btns & BTN_X)
        {
            if (dpadUp)
                // servo2Pos = std::min(180, servo2Pos + servoStep);
                servoForward(servo[2], servo2Pos, servoStep, 18);
            if (dpadDown)
                servoBackward(servo[2], servo2Pos, servoStep, 18);
            // servo2Pos = std::max(0, servo2Pos - servoStep);
        }
    }

    // --- Core Initializer loop() ---
    void printRawBytes(const uint8_t *data, size_t len)
    {
        Serial.print("   Rx: ");
        for (size_t i = 0; i < len; ++i)
            Serial.printf("%02x", data[i]);
        Serial.println();
    }
    void core1()
    {
        static Joypad cntr = {};
        static uint8_t rawBuf[sizeof(Joypad)];
        static unsigned long lastPacketTime = 0;
        static unsigned long lastLedTime = 0;
        int len = net::udp::socket.udp.parsePacket();

        // Serial.print("[core1]");
        if (len >= sizeof(Joypad))
        {
            net::udp::socket.udp.read(rawBuf, sizeof(rawBuf));
            cntr = *reinterpret_cast<Joypad *>(rawBuf);

            printRawBytes(rawBuf, sizeof(rawBuf));

            // actual logic
            coreAction(cntr);

            digitalWrite(sys::hw.LED, HIGH);
        }
        else
        {
            if (millis() - lastLedTime > 200)
            {
                lastLedTime = millis();
                digitalWrite(sys::hw.LED, HIGH);
            }
            else
            {
                digitalWrite(sys::hw.LED, LOW);
            }
        }
        coreActionAsync(cntr);
    }
}
