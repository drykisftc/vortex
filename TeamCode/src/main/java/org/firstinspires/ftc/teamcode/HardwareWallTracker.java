package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

public class HardwareWallTracker extends HardwareVortex {

    ModernRoboticsI2cRangeSensor leftRange = null;
    ModernRoboticsI2cRangeSensor rightRange = null;

    public void init(HardwareMap ahwMap) {

        // Initialize base Motor and Servo objects
        super.init(ahwMap);

        leftRange =  hwMap.get(ModernRoboticsI2cRangeSensor.class, "leftRange");
        rightRange =  hwMap.get(ModernRoboticsI2cRangeSensor.class, "rightRange");

    }
}
