package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

public class HardwareLineTracker extends HardwareVortex {

    OpticalDistanceSensor [] sensorArray = null;

    int arraySize = 1;

    public void init(HardwareMap ahwMap, int numbOfSensors) {

        // Initialize base Motor and Servo objects
        super.init(ahwMap);

        sensorArray = new OpticalDistanceSensor[numbOfSensors];
        arraySize = numbOfSensors;

        for (int i = 0; i < numbOfSensors; i++) {
            sensorArray[i] = hwMap.opticalDistanceSensor.get("ods"+Integer.toString(i));
        }
    }
}
