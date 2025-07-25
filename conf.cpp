#include "src/lib/lib.h"

void conf_pins()
{
    // net
    net::sta::addConnections(2, (const char *[][2]){{"goochi ku", "12345679"}, {"tenet", "1219@121"}, {"goochi-ku", "1219@1219"}});

    // Pins
    pinMode(sys::hw.BUZZER, OUTPUT);
    pinMode(sys::hw.LED, OUTPUT);

    register_motors(1, (int[][3]){{25, 26, 27}});
    // register_servos(3, (int[]){21, 22, 23});
    // register_servos(3, (int[]){4, 19, 23});
    register_servos(3, (int[]){5, 18, 19});
    register_steppers(1, (int[][2]){{16, 17}});

    // Assign Coore Logic
    lgc::add(lgc::core1, "core1");
    lgc::add(lgc::core2, "core2");
    lgc::add(lgc::core3, "core3");
    lgc::assign("core1");
}