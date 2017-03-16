package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by andrewwang on 3/15/17.
 */

public class HardwareCompass extends HardwareBase {
    ModernRoboticsI2cCompassSensor Compass = null;

    public void init(HardwareMap ahwMap) {

        // Initialize base Motor and Servo objects
        super.init(ahwMap);
        Compass = (ModernRoboticsI2cCompassSensor) hwMap.compassSensor.get("compass");
    }
}
