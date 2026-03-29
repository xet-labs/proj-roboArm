#include "lgc.h"
#include "../lib/lib.h"
#include <ESP32Servo.h>
#include <AccelStepper.h>

namespace lgc
{
    namespace Ps3Btn = controller::Ps3::Button;
    // Using Joypad state and buttons from utils.h (libXetArduino)

    const int moveInterval = 15;

    // --runtime var--

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

    void stats(const controller::Ps3::State &Ps3)
    {
        // constant stats display may cause jittery movements, so during prod
        // return;
        
        const char *dcState;
        if (Ps3.btns & Ps3Btn::RB)
            dcState = "CW";
        else if (Ps3.btns & Ps3Btn::LB)
            dcState = "CCW";
        else
            dcState = "STOP";

        Serial.printf("[JOY] Soldr: %d, Elbow: %d, Wrist: %d | Stepper: %d | DC: %s | servoStep: %d\n",
                      servo0Pos, servo1Pos, servo2Pos,
                      stepper0Pos, dcState, servoStep);
    }

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

    void coreAct(const controller::Ps3::State &Ps3)
    {
        // Servos speed
        servoStep = Ps3.triggerRT > 0 ? map(Ps3.triggerRT, 0, 255, servoStepMin, servoStepMax) : servoStepDef;
        // Dc Motor Speed
        motor[0].setSpeed(Ps3.triggerLT > 0 ? Ps3.triggerLT : dcSpeedDef);

        stepper[0].setMaxSpeed(500);
        stepper[0].setAcceleration(200);

        // === DPAD Controls ===
        dpadUp = Ps3.btns & Ps3Btn::DPAD_UP;
        dpadDown = Ps3.btns & Ps3Btn::DPAD_DOWN;
        dpadLeft = Ps3.btns & Ps3Btn::DPAD_LEFT;
        dpadRight = Ps3.btns & Ps3Btn::DPAD_RIGHT;

        // === Stepper Movement (Still using axisLX) ===
        // int stepTarget = map(Ps3.axisLX, -32768, 32767, -200, 200);
        // stepper[0].moveTo(stepTarget);
        // stepper[0].run();

        // === DC Motor Control (RB = CW, LB = CCW) ===
        switch (Ps3.btns & (Ps3Btn::RB | Ps3Btn::LB))
        {
        case Ps3Btn::RB:
            motor[0].forward();
            break;

        case Ps3Btn::LB:
            motor[0].backward();
            break;

        case (Ps3Btn::RB | Ps3Btn::LB): // both pressed
            motor[0].stop();    // or do something else
            break;

        default:
            motor[0].stop();
            break;
        }

        stats(Ps3);
    }

    void coreActAsync(const controller::Ps3::State &Ps3)
    {
        // Stepper movement

        if (dpadLeft)
        {
            stepperBackward(stepper[0], stepper0Pos, stepperStep, 14);
            stats(Ps3);
        }
        if (dpadRight)
        {
            stepperForward(stepper[0], stepper0Pos, stepperStep, 14);
            stats(Ps3);
        }
        // soldr control (A + UP/DOWN)
        if (Ps3.btns & Ps3Btn::A)
        {
            if (dpadUp)
            {
                servoForward(servo[0], servo0Pos, servoStep, 18);
                stats(Ps3);

            }
            if (dpadDown)
            {
                servoBackward(servo[0], servo0Pos, servoStep, 18);
                stats(Ps3);
            }
        }

        // Elbow control (B + UP/DOWN)
        if (Ps3.btns & Ps3Btn::B)
        {
            if (dpadUp)
            {
                servoBackward(servo[1], servo1Pos, servoStep, 18);
                stats(Ps3);
            }
            if (dpadDown)
            {
                servoForward(servo[1], servo1Pos, servoStep, 18);
                stats(Ps3);
            }
        }

        // Head control (X + UP/DOWN)
        if (Ps3.btns & Ps3Btn::X)
        {
            if (dpadUp)
            {
                servoForward(servo[2], servo2Pos, servoStep, 18);
                stats(Ps3);
            }
            if (dpadDown)
            {
                servoBackward(servo[2], servo2Pos, servoStep, 18);
                stats(Ps3);
            }
        }
    }

    // --- Core Initializer loop() ---
    void core1()
    {
        static controller::Ps3::State cntr = {};
        static uint8_t rawBuf[sizeof(cntr)];
        static unsigned long lastPacketTime = 0;
        int len = net::udp::socket.udp.parsePacket();

        if (len >= sizeof(controller::Ps3::State))
        {
            net::udp::socket.udp.read(rawBuf, sizeof(rawBuf));
            cntr = *reinterpret_cast<controller::Ps3::State*>(rawBuf);

            // show received bytes
            // util::printBytes(rawBuf, sizeof(rawBuf));
            // controller::Ps3::printState(cntr, rawBuf, sizeof(rawBuf));

            // controller logic
            coreAct(cntr);

            if (!sys::hw.LED_LOCK) digitalWrite(sys::hw.LED, 1);
        }
        else if (!sys::hw.LED_LOCK) digitalWrite(sys::hw.LED, 0);

        // asynchronous controller logic
        coreActAsync(cntr);
    }
}
