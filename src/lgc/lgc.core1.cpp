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

    bool demoMode = true;

    void runDemoSequence(bool reset = false)
    {
        static int demoState = 0;
        static unsigned long stateStartTime = 0;
        static float startPos[3] = {90.0f, 90.0f, 90.0f};

        if (reset) {
            demoState = 0;
            stateStartTime = 0;
            motor[0].stop();
            return;
        }

        unsigned long now = millis();

        const int NUM_STATES = 13;
        // { Soldier/Shoulder, Elbow, Wrist, StepperBase, DCMotor (-1=CCW, 0=Stop, 1=CW) }
        static const int targets[NUM_STATES][5] = {
            {90,  90,  90,    0,  0},    // 0: Centered Rest
            {90,  90,  90,  150,  0},    // 1: Swing Base Right (Arm raised)
            {60,  140, 90,  150,  0},    // 2: Reach Down to object
            {60,  140, 90,  150,  1},    // 3: SPIN DC CW (Screw/Grab)
            {120, 90,  90,  150,  0},    // 4: Lift Arm high, Stop DC
            {120, 90, 150,  150,  0},    // 5: **Wrist Fwd/Up** (Only wrist moves)
            {120, 90,  30,  150,  0},    // 6: **Wrist Bkwd/Down** (Only wrist moves)
            {120, 90,  90,  150,  0},    // 7: **Wrist Center**
            {120, 90,  90, -150,  0},    // 8: Swing Base Left
            {60,  140, 90, -150,  0},    // 9: Reach Down to drop zone
            {60,  140, 90, -150, -1},    // 10: SPIN DC CCW (Unscrew/Release)
            {120, 90,  90, -150,  0},    // 11: Lift Arm high, Stop DC
            {90,  90,  90,    0,  0}     // 12: Return to Centered Rest
        };
        
        // Time in milliseconds to smoothly travel to the state
        static const unsigned long travelTime[NUM_STATES] = {
            2000,  // 0: to Center 
            2000,  // 1: to Swing Right
            1500,  // 2: to Reach Down
            1500,  // 3: to Spin DC CW (stationary wait for 1.5s)
            1500,  // 4: to Lift Arm
            1000,  // 5: to Wrist Fwd
            1200,  // 6: to Wrist Bkwd
            1000,  // 7: to Wrist Center
            2500,  // 8: to Swing Left
            1500,  // 9: to Reach Down
            1500,  // 10: to Spin DC CCW (stationary wait for 1.5s)
            1500,  // 11: to Lift Arm
            2000   // 12: to Center
        };

        if (stateStartTime == 0) {
            stateStartTime = now;
            // Lock in the starting position so it never jerks!
            startPos[0] = servo0Pos;
            startPos[1] = servo1Pos;
            startPos[2] = servo2Pos;

            stepper[0].setMaxSpeed(400); // Smoother stepper
            stepper[0].setAcceleration(100);
            stepper[0].moveTo(targets[demoState][3]);

            // Set DC motor state immediately for this keyframe
            int dcTarget = targets[demoState][4];
            if (dcTarget > 0) {
                motor[0].setSpeed(dcSpeedDef);
                motor[0].forward();
            } else if (dcTarget < 0) {
                motor[0].setSpeed(dcSpeedDef);
                motor[0].backward();
            } else {
                motor[0].stop();
            }
        }

        unsigned long elapsed = now - stateStartTime;
        unsigned long total = travelTime[demoState];

        if (elapsed <= total) {
            // Easing function (Smoothstep) to decelerate/accelerate butter smoothly!
            float progress = (float)elapsed / total;
            progress = progress * progress * (3.0f - 2.0f * progress); 

            servo0Pos = startPos[0] + (targets[demoState][0] - startPos[0]) * progress;
            servo1Pos = startPos[1] + (targets[demoState][1] - startPos[1]) * progress;
            servo2Pos = startPos[2] + (targets[demoState][2] - startPos[2]) * progress;

            servo[0].write(servo0Pos);
            servo[1].write(servo1Pos);
            servo[2].write(servo2Pos);
        } else {
            // Arrived at target. Snap accurately.
            servo0Pos = targets[demoState][0];
            servo1Pos = targets[demoState][1];
            servo2Pos = targets[demoState][2];
            servo[0].write(servo0Pos);
            servo[1].write(servo1Pos);
            servo[2].write(servo2Pos);

            if (stepper[0].distanceToGo() == 0) {
                // Pause for 400ms before transitioning to next state naturally
                if (elapsed > total + 400) {
                    demoState = (demoState + 1) % NUM_STATES;
                    stateStartTime = 0;
                }
            }
        }
        
        stepper[0].run();
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


        bool newPacket = false;
        
        while (Serial.available() >= 14) 
        {
            if (Serial.peek() == 0xAA) {
                Serial.read(); // Consume 0xAA
                if (Serial.peek() == 0x55) {
                    Serial.read(); // Consume 0x55
                    Serial.readBytes((uint8_t*)rawBuf, sizeof(rawBuf));
                    cntr = *reinterpret_cast<controller::Ps3::State*>(rawBuf);
                    newPacket = true;
                } else {
                    break;
                }
            } else {
                break;
            }
        }

        if (newPacket)
        {
            // Toggle demo mode with START button with edge detection
            static bool lastStartBtn = false;
            bool currentStartBtn = (cntr.btns & Ps3Btn::START);
            
            if (currentStartBtn && !lastStartBtn) {
                demoMode = !demoMode;
                if (demoMode) {
                    Serial.println("[DEMO] Mode Activated!");
                    runDemoSequence(true); // Always reset demo sequence when toggling ON
                } else {
                    Serial.println("[DEMO] Mode Deactivated!");
                }
            }
            lastStartBtn = currentStartBtn;
            
            // Safety abort on any other button input
            if (demoMode) {
                // If any button is pressed (excluding START), deactivate immediately
                if (cntr.btns & ~(Ps3Btn::START)) {
                    demoMode = false;
                    Serial.println("[DEMO] Aborted by manual input!");
                }
            }

            // show received bytes
            // util::printBytes(rawBuf, sizeof(rawBuf));
            // controller::Ps3::printState(cntr, rawBuf, sizeof(rawBuf));

            // Execute standard controller logic only if we are in manual mode
            if (!demoMode) {
                coreAct(cntr);
            }

            if (!sys::hw.LED_LOCK) digitalWrite(sys::hw.LED, 1);
        }
        else if (!sys::hw.LED_LOCK) digitalWrite(sys::hw.LED, 0);

        // asynchronous controller logic
        if (demoMode) {
            runDemoSequence();
            
            // Display tracking stats less frequently dynamically inside demo sequence
            static unsigned long lastDemoStat = 0;
            if (millis() - lastDemoStat > 200) {
                lastDemoStat = millis();
                stats(cntr); 
            }
        } else {
            coreActAsync(cntr);
        }
    }
}
