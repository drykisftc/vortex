package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.DecimalFormat;

public class GyroTracker extends Excecutor {

    ModernRoboticsI2cGyro gyro = null;

    DcMotor leftWheel = null;
    DcMotor rightWheel = null;

    private int bufferSize = 10;
    private double[] skewAngleBuffer = null;
    private double[] powerBuffer = null;
    private int bufferIndex = 0;

    private int targetHeading =0;

    /*
    adjust to the correct sensitivity for each robot
     */
    double skewAngelPowerGain = 1.0/90.0;
    double skewAngelTolerance = 0;


    public GyroTracker(ModernRoboticsI2cGyro leftO,
                       DcMotor leftW,
                       DcMotor rightW,
                       int bufferS){
        gyro = leftO;
        leftWheel = leftW;
        rightWheel = rightW;
        bufferSize = bufferS;
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init() {

        skewAngleBuffer = new double [bufferSize];
        powerBuffer = new double[bufferSize];

        state =0;
    }

    public boolean goStraight (int target, double power) {
        targetHeading = target;

        boolean doNothing = true;

        // compute power delta
        int heading = gyro.getHeading();
        double delta = VortexUtils.getAngleError(target, heading);
        double deltaPower = 0.0;
        if (Math.abs(delta) > skewAngelTolerance) {
            deltaPower = skewAngelPowerGain * delta;
            doNothing = false;
        }

        // move motor
        leftWheel.setPower(Range.clip(power - deltaPower, -1, 1));
        rightWheel.setPower(Range.clip(power + deltaPower, -1, 1));

        // save history
        skewAngleBuffer[bufferIndex] = delta;
        powerBuffer[bufferIndex] = deltaPower;
        bufferIndex++;
        if (bufferIndex >= bufferSize) {
            bufferIndex = 0;
        }

        if (reporter != null) {
            reporter.addData("Heading angle       =", "%3d", heading);
            reporter.addData("Heading angle skew  =", "%.2f vs %.2f", delta, skewAngelTolerance);
            reporter.addData("Heading power       =", "%.2f", power);
            reporter.addData("Heading delta power =", "%.2f", deltaPower);
        }

        return doNothing;
    }

    public boolean turn (int target, double minPower) {
        targetHeading = target;

        boolean doNothing = true;

        // compute power delta
        int heading = gyro.getHeading();
        double delta = VortexUtils.getAngleError(target, heading);
        double deltaPower = 0.0;
        if (Math.abs(delta) > skewAngelTolerance) {
            deltaPower = skewAngelPowerGain * delta;
            doNothing = false;
        }

        // move motor
        if (deltaPower !=0) {
            leftWheel.setPower(Range.clip(Math.min(minPower * (-deltaPower) / Math.abs(deltaPower), -deltaPower), -1, 1));
            rightWheel.setPower(Range.clip(Math.max(minPower * deltaPower / Math.abs(deltaPower), deltaPower), -1, 1));
        } else {
            leftWheel.setPower(0.0);
            rightWheel.setPower(0.0);
        }

        // save history
        skewAngleBuffer[bufferIndex] = delta;
        powerBuffer[bufferIndex] = deltaPower;
        bufferIndex++;
        if (bufferIndex >= bufferSize) {
            bufferIndex = 0;
        }

        if (reporter != null) {
            reporter.addData("Heading angle       =", "%3d", heading);
            reporter.addData("Heading angle skew  =", "%.2f vs %.2f", delta, skewAngelTolerance);
            reporter.addData("Heading power       =", "%.2f", minPower);
            reporter.addData("Heading delta power =", "%.2f", deltaPower);
        }

        return doNothing;
    }

}
