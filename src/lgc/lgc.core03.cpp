#include "lgc.h"
// #include "../lib/lib.h"

namespace lgc
{
    // State Variables
    bool autoMode = false;
    int stepperPos = 0;

    void core3()
    {
        Serial.println("[core1] logic running");
        
        auto &tcpSrv = net::tcp::server;

        tcpSrv.listen();
        if (!tcpSrv.hasClient())
            return;

        String command = tcpSrv.readLine();
        command.trim();
        Serial.println("Received: " + command);

        if (command.startsWith("BASE:"))
            stepper[0].moveTo(command.substring(5).toInt());
        else if (command.startsWith("S1:"))
            servo[0].write(command.substring(3).toInt());
        else if (command.startsWith("S2:"))
            servo[1].write(command.substring(3).toInt());
        else if (command.startsWith("S3:"))
            servo[2].write(command.substring(3).toInt());
        else if (command.startsWith("SCREW:"))
            command.substring(6) == "ON" ? motor[0].forward() : motor[0].stop();
        else if (command.startsWith("AUTO:"))
            autoMode = (command.substring(5) == "ON");

        if (autoMode)
        {
#if ENABLE_VL53L0X
            VL53L0X_RangingMeasurementData_t measure;
            sensor1.rangingTest(&measure, false);
            if (measure.RangeStatus != 4 && measure.RangeMilliMeter < 50)
            {
                Serial.println("Hole Detected - Auto Screw");
                motor[0].forward();
                delay(2000);
                motor[0].stop();
                autoMode = false;
            }
#else
            Serial.println("[AUTO MODE IGNORED] VL53L0X disabled");
            autoMode = false;
#endif
        }

        stepper[0].run();
    }

}