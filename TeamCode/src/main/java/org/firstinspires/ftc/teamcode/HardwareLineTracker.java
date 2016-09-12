package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

public class HardwareLineTracker extends HardwareVortex {

    OpticalDistanceSensor leftODS = null;
    OpticalDistanceSensor rightODS = null;
    OpticalDistanceSensor middleODS = null;

    public void init(HardwareMap ahwMap) {

        // Initialize base Motor and Servo objects
        super.init(ahwMap);

        leftODS = hwMap.opticalDistanceSensor.get("leftODS");
        rightODS = hwMap.opticalDistanceSensor.get("rightODS");
        middleODS = hwMap.opticalDistanceSensor.get("middleODS");

    }
}
