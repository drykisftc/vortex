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
    public  int breakDistance = 500; // slow down before complete stop
    private double breakPower = 0.15;

    /*
    adjust to the correct sensitivity for each robot
     */
    private double minTurnSpeed = 1.0;
    private double maxTurnSpeed = 10;
    private double minAnglePowerStepSize = 0.0005;
    private int flipCountLimit = 1;
    
    private long lastLogTimeStamp = 0;
    final private int sameplingInteval = 100;

    private int landMarkPosition = 0;
    private int landMarkAngle = 0;

    private int headingHits  = 0;
    private int headingHitsLimit = 2;

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
        report("GyroTracker", "init....");
        
        state =0;
    }

    @Override
    public void start (int state ) {
        super.start(state);
        gyro.resetZAxisIntegrator();
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

        //  complete state checking
        if ( deltaPower  ==  0.0) {
            headingHits++;
        } else {
            headingHits = 0;
        }

        if (headingHits >= headingHitsLimit) {
            boolNoTurning = true;
        }

        // save history
        saveHistory(delta, deltaPower);

        if (reporter != null) {
            reporter.addData("Heading angle       =", "%3d", heading);
            reporter.addData("Heading target      =", "%.3f", targetHeading);
            reporter.addData("Heading delta       =", "%.2f", delta);
            reporter.addData("Heading tolerance   =", "%.2f", skewTolerance);
            reporter.addData("Heading gain        =", "%.2f", skewPowerGain);
            reporter.addData("Minimum turn power  =", "%.2f", minTurnPower);
            reporter.addData("Heading power       =", "%.2f", power);
            reporter.addData("Heading turn power  =", "%.2f", deltaPower);
            reporter.addData("Landmark            =", landMarkPosition);
        }

        return boolNoTurning;
    }

    private void adjustMinTurnPower(double currentDelta) {
        if ( Math.abs(currentDelta) > skewTolerance) {
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
                minTurnPower -= minAnglePowerStepSize;
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
                            int targetDistance,
                            int startState, int endState) {

        // check target distance sign
        targetDistance = Math.abs(targetDistance);

        // get traveling distance
        int lD = leftWheel.getCurrentPosition();
        int rD = rightWheel.getCurrentPosition();
        int d = (lD+rD)/2;
        skewPowerGain = gain;
        int travelDistance = d - landMarkPosition;
        double breakingP = Math.abs(breakPower);
        if (power < 0) { // if move backward
            travelDistance *= -1;
            breakingP *= -1;
        }

        // move
        if (travelDistance < targetDistance) {
            if (targetDistance - travelDistance < Math.abs(breakDistance)) {
                // slow down when close to target
                maintainHeading(heading, breakingP);
                return startState;
            } else {
                maintainHeading(heading, power);
                return startState;
            }
        }

        // stop and return next state
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
        landMarkPosition = (lD+rD)/2;
        return endState;
    }

    public int getWheelLandmarkOdometer () {
        int lD = leftWheel.getCurrentPosition();
        int rD = rightWheel.getCurrentPosition();
        int d = (lD+rD)/2;
        return d- landMarkPosition;
    }

    public void setWheelLandmark () {
        int lD = leftWheel.getCurrentPosition();
        int rD = rightWheel.getCurrentPosition();
        int d = (lD+rD)/2;
        landMarkPosition = d;
    }

    public int getWheelLandmark () {
        return landMarkPosition;
    }

    public void stopWheels() {
        leftWheel.setPower(0.0);
        rightWheel.setPower(0.0);
    }
    public void waggleWheels (double power ) {
        leftWheel.setPower(power);
        rightWheel.setPower(-1.0*power);
    }

}
