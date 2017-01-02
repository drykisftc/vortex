package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class HardwareWallTracker extends HardwareBase {

    ModernRoboticsI2cRangeSensor sonicRange = null;

    public void init(HardwareMap ahwMap) {

        // Initialize base Motor and Servo objects
        super.init(ahwMap);

        sonicRange =  hwMap.get(ModernRoboticsI2cRangeSensor.class, "sonicRange");

    }

    public double getDistance () {
        return sonicRange.getDistance(DistanceUnit.CM);
    }

}
