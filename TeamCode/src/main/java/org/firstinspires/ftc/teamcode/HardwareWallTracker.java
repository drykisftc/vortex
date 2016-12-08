package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class HardwareWallTracker extends HardwareBase {

    Servo sonicArm = null;
    ModernRoboticsI2cRangeSensor sonicRange = null;

    double parkingPosition = 0.5;
    double movingStep = 0.05;
    double leftMax = 1.0;
    double rightMax = 0.0;

    public void init(HardwareMap ahwMap) {

        // Initialize base Motor and Servo objects
        super.init(ahwMap);

        sonicArm =  hwMap.servo.get("sonicArm");
        sonicRange =  hwMap.get(ModernRoboticsI2cRangeSensor.class, "sonicRange");

    }

    public void park() {
        sonicArm.setPosition(parkingPosition);
    }

    public void moveSonicArmToLeft() {
        sonicArm.setPosition(Range.clip(sonicArm.getPosition() + movingStep, 0.0, 1.0));
    }

    public void moveSonicArmToMaxLeft() {
        sonicArm.setPosition(leftMax);
    }

    public void moveSonicArmToRight() {
        sonicArm.setPosition(Range.clip(sonicArm.getPosition() - movingStep, 0.0, 1.0));
    }

    public void moveSonicArmToMaxRight() {
        sonicArm.setPosition(rightMax);
    }

    public double getDistance (double position) {
        sonicArm.setPosition(position);
        return sonicRange.getDistance(DistanceUnit.CM);
    }

    public double getDistance () {
        return sonicRange.getDistance(DistanceUnit.CM);
    }

}
