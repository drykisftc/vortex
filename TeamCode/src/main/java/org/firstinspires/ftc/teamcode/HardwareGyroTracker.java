package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareGyroTracker extends HardwareVortex {

    ModernRoboticsI2cGyro gyro = null;

    public void init(HardwareMap ahwMap) {

        // Initialize base Motor and Servo objects
        super.init(ahwMap);

        gyro =  (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");

        gyro.calibrate();

    }
}
