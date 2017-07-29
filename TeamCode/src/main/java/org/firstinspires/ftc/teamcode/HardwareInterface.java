package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.Device;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by andrewwang on 3/15/17.
 */

public class HardwareInterface extends HardwareBase {
    DeviceInterfaceModule Interface = null;

    public void init(HardwareMap ahwMap) {

        // Initialize base Motor and Servo objects
        super.init(ahwMap);
        //replace Placeholder with the DeviceInterfaceModule's address
        Interface = (DeviceInterfaceModule) hwMap.compassSensor.get("Placeholder");
    }
}
