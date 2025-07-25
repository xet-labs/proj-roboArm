#include "lgc.h"
// #include "../lib/lib.h"

namespace lgc
{
    void core2()
    {
        static unsigned long lastToggle = 0;
        static bool direction = false;

        unsigned long now = millis();

        // Toggle every 2 seconds
        if (now - lastToggle > 2000)
        {
            lastToggle = now;
            direction = !direction;

            // Stepper move
            int target = direction ? 200 : 0;
            stepper[0].moveTo(target);
            Serial.printf("[TEST] Stepper moveTo(%d)\n", target);

            // Servo sweep
            int angle = direction ? 150 : 30;
            servo[0].write(angle);
            servo[1].write(180 - angle);
            servo[2].write(angle);
            Serial.printf("[TEST] Servo angle: %d\n", angle);

            // DC motor toggle
            if (direction)
            {
                motor[0].forward();
                Serial.println("[TEST] DC Motor forward");
            }
            else
            {
                motor[0].backward();  // FIXED: use backward instead of reverse
                Serial.println("[TEST] DC Motor backward");
            }
        }

        // Run stepper motor
        stepper[0].run();
    }
}
