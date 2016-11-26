package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class GyroTracker extends Tracker {

    ModernRoboticsI2cGyro gyro = null;
    private DcMotor leftWheel = null;
    private DcMotor rightWheel = null;

    private int bufferSize = 20;
    private double[] skewAngleBuffer = null;
    private double[] powerBuffer = null;
    private int bufferIndex = 0;

    private double targetHeading =0;

    /*
    adjust to the correct sensitivity for each robot
     */
    private double minTurnSpeed = 1.0;
    private double maxTurnSpeed = 10;
    private double minAnglePowerStepSize = 0.02;
    private int flipCountLimit = 1;
    
    private long lastLogTimeStamp = 0;
    final private int sameplingInteval = 100;

    private int landMarkPosition = 0;
    private int landMarkAngle = 0;

    public GyroTracker(ModernRoboticsI2cGyro g,
                       DcMotor leftW,
                       DcMotor rightW,
                       int bufferS){
        gyro = g;
        leftWheel = leftW;
        rightWheel = rightW;
        bufferSize = bufferS;
        skewAngleBuffer = new double [bufferSize];
        powerBuffer = new double[bufferSize];
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init() {
        if (reporter != null) {
            reporter.addData("GyroTracker", "init....");
        }
        state =0;
    }

    @Override
    public void start (int state ) {
        super.start(state);
        int lD = leftWheel.getCurrentPosition();
        int rD = rightWheel.getCurrentPosition();
        landMarkPosition = Math.min(lD, rD);
        landMarkAngle = gyro.getHeading();
    }

    public void setLandMarkAngle (int angle) {
        landMarkAngle = angle;
    }

    public void setLandMarkPosition ( int position) {
        landMarkPosition = position;
    }

    /**
     *
     * @param target heading
     * @param power moving power
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
            reporter.addData("Heading tolerance   =", "%.2f", skewTolerance);
            reporter.addData("Heading power       =", "%.2f", power);
            reporter.addData("Heading delta power =", "%.2f", deltaPower);
            reporter.addData("Heading gain        =", "%.2f", skewPowerGain);
            reporter.addData("Minimum turn power  =", "%.2f", minTurnPower);
        }

        return boolNoTurning;
    }

    private void adjustMinTurnPower(double currentDelta) {
        if (currentDelta > skewTolerance) {
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
                lastV = v;
            }

            // adjust minimum turn force
            double deltaChange = maxV - minV;
            if (deltaChange < minTurnSpeed) {
                // robot could not turn, boost min turn power to over come the friction
                minTurnPower += minAnglePowerStepSize;
            } else if (flipCount >= flipCountLimit ) {
                // robot always over-compensated, tune down the min turn power
                minTurnPower -= minAnglePowerStepSize*0.618;
            }
        }
    }

    private  void saveHistory (double delta, double deltaPower){
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

    public int goStraight ( double heading, double gain, double power,
                            int deltaDistance,
                            int startState, int endState) {
        // get motor distance
        int lD = leftWheel.getCurrentPosition();
        int rD = rightWheel.getCurrentPosition();
        int d = Math.min(lD, rD);

        if ( d - landMarkPosition < deltaDistance) {
            skewPowerGain = gain;
            maintainHeading(heading, power);
            return startState;
        }
        // go to next state
        landMarkPosition = d;
        leftWheel.setPower(0.0);
        rightWheel.setPower(0.0);
        return endState;
    }

    public int turn ( double heading, double sensitivity, double power,
                      int startState, int endState) {
        skewPowerGain = sensitivity;
        if (maintainHeading(heading, power) != true) {
            return startState;
        }
        /* got to next state */
        leftWheel.setPower(0.0);
        rightWheel.setPower(0.0);
        int lD = leftWheel.getCurrentPosition();
        int rD = rightWheel.getCurrentPosition();
        landMarkPosition = Math.min(lD, rD);
        return endState;
    }

    public int getWheelLandmarkOdometer () {
        int lD = leftWheel.getCurrentPosition();
        int rD = rightWheel.getCurrentPosition();
        int d = Math.min(lD, rD);
        return d- landMarkPosition;
    }

    public void setWheelLandmark () {
        int lD = leftWheel.getCurrentPosition();
        int rD = rightWheel.getCurrentPosition();
        int d = Math.min(lD, rD);
        landMarkPosition = d;
    }


}
