package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class GyroTracker extends Excecutor {

    ModernRoboticsI2cGyro gyro = null;

    DcMotor leftWheel = null;
    DcMotor rightWheel = null;

    private int bufferSize = 10;
    private double[] skewAngleBuffer = null;
    private double[] powerBuffer = null;
    private int bufferIndex = 0;

    private double targetHeading =0;

    /*
    adjust to the correct sensitivity for each robot
     */
    double minTurnPower = 0.05;  // change this value for different robot to compensate the friction
    double skewAngelPowerGain = 1.0/180.0;
    double skewAngelTolerance = 0;
    private double minTurnSpeed = 1.0;
    private double maxTurnSpeed = 10;
    private double minAnglePowerStepSize = 0.02;


    private long lastLogTimeStamp = 0;
    final private int sameplingInteval = 100;

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
     * @param target
     * @param power
     * @return true if no heading correction
     */
    public boolean maintainHeading(double target, double power) {

        targetHeading = VortexUtils.normalizeHeading(target);

        boolean boolNoTurning = false;
        int heading = gyro.getHeading();
        double delta = VortexUtils.getAngleError(targetHeading, heading);

        // compute power delta
        double deltaPower = computeTurnPower(delta);
        leftWheel.setPower(Range.clip(power - deltaPower, -1, 1));
        rightWheel.setPower(Range.clip(power + deltaPower, -1, 1));

        // move motor
        if ( deltaPower  ==  0.0) {
            boolNoTurning = true;
        }

        // save history
        saveHistory(delta, deltaPower);

        if (reporter != null) {
            reporter.addData("Heading angle       =", "%3d", heading);
            reporter.addData("Heading target      =", "%.3f", targetHeading);
            reporter.addData("Heading delta       =", "%.2f", delta);
            reporter.addData("Heading tolerance   =", "%.2f", skewAngelTolerance);
            reporter.addData("Heading power       =", "%.2f", power);
            reporter.addData("Heading delta power =", "%.2f", deltaPower);
            reporter.addData("Heading gain        =", "%.2f", skewAngelPowerGain);
            reporter.addData("Minimum turn power  =", "%.2f", minTurnPower);
        }

        return boolNoTurning;
    }

    protected void adjustMinTurnPower(double currentDelta) {
        if (currentDelta > skewAngelTolerance ) {
            double minV = 1000;
            double maxV = -1000;
            double lastV = 0;
            int flipCount =0;
            for (int i = 0; i < bufferIndex; i++) {
                double v = skewAngleBuffer[i];
                minV = Math.min(minV, v);
                maxV = Math.max(maxV, v);
                if (lastV * v < 0) {
                    flipCount ++;
                }
            }

            // adjust minimum turn force
            double deltaChange = maxV - minV;
            if (deltaChange < minTurnSpeed) {
                // robot could not turn, boost min turn power to over come the friction
                minTurnPower += minAnglePowerStepSize;
            } else if (flipCount >=1 ) {
                // robot always over-compensated, tune down the min turn power
                minTurnPower -= minAnglePowerStepSize*0.618;;
            }
        }
    }

    protected  void saveHistory (double delta, double deltaPower){
        long ts = System.currentTimeMillis();
        if ( ts - lastLogTimeStamp > sameplingInteval) {
            skewAngleBuffer[bufferIndex] = delta;
            powerBuffer[bufferIndex] = deltaPower;
            bufferIndex++;
            if (bufferIndex >= bufferSize) {
                bufferIndex = 0;
            }
            lastLogTimeStamp= ts;
            adjustMinTurnPower(delta);
        }
    }

    /**
     *
     * @param deltaHeading
     * @return delta power = (+/-)minTurnPower + gain * skew
     */
    protected  double computeTurnPower (double deltaHeading) {
        double deltaPower = 0.0;
        if (Math.abs(deltaHeading) > skewAngelTolerance) {
            deltaPower = skewAngelPowerGain * deltaHeading;
            // always apply minimum force to compensate the friction
            if (deltaPower > 0.0) {
                deltaPower += minTurnPower;
            } else if (deltaPower < 0.0  ) {
                deltaPower -= minTurnPower;
            }
        }
        return deltaPower;
    }

}
