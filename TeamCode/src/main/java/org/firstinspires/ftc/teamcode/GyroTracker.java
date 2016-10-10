package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.DecimalFormat;

public class GyroTracker {

    ModernRoboticsI2cGyro gyro = null;

    DcMotor leftWheel = null;
    DcMotor rightWheel = null;

    // state machine
    public int state = 0;

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

    Telemetry reporter = null;

    private static DecimalFormat df3 = new DecimalFormat(".###");

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

    /**
     *
     * @param startState, The initial state of wall follower
     */
    public void start(int startState) {

        state = startState;
    }

    public boolean goStraight (int target, double power) {
        targetHeading = target;

        double delta = VortexUtils.getAngleError(target,gyro.getHeading());
        skewAngleBuffer[bufferIndex] = delta;
        double deltaPower = skewAngelPowerGain * delta;
        boolean doNothing = true;

        if (Math.abs(delta) > skewAngelTolerance) {
            // move motor
            leftWheel.setPower(Range.clip(power - deltaPower, -1, 1));
            rightWheel.setPower(Range.clip(power + deltaPower, -1, 1));
            doNothing = false;
        } else {
            deltaPower = 0.0;
        }

        powerBuffer[bufferIndex] = deltaPower;
        bufferIndex++;
        if (bufferIndex >= bufferSize) {
            bufferIndex = 0;
        }

        if (reporter != null) {
            reporter.addData("Heading angle skew", "%.2f vs %.2f", delta, skewAngelTolerance);
            reporter.addData("Heading power", "%.2f", deltaPower);
        }

        return doNothing;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    public void stop() {
        leftWheel.setPower(0.0);
        rightWheel.setPower(0.0);
    }

    public void setReporter (Telemetry t){
        reporter = t;
    }


}
