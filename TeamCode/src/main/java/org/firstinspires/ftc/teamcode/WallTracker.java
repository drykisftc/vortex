package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.text.DecimalFormat;

public class WallTracker extends Tracker {

    HardwareWallTracker wallTrackerHW = null;

    DcMotor leftWheel = null;
    DcMotor rightWheel = null;

    // state machine
    public int state = 0;

    private int bufferSize = 6;
    private double[] distanceBuffer = null;
    private double[] powerBuffer = null;
    private int bufferIndex = 0;

    public double targetDistance = 5.0; // unit cm
    private double lastDirection = 0.0d;
    private double lostDistance = 100;

    double[] dist2PowerLUT = {0.0f, 0.05f, 0.15f, 0.18f, 0.20f,
            0.22f, 0.24f, 0.26f, 0.28f, 0.30f, 0.32f, 0.34f, 0.36f,
            0.38f, 0.42f, 0.46f, 0.50f, 0.54f, 0.58f, 0.62f, 0.66f,
            0.70f, 0.74f, 0.78f, 0.82f, 0.86f, 0.90f, 0.94f, 0.98f, 1.00f};

    private static DecimalFormat df3 = new DecimalFormat(".###");

    // jam detection
    protected JammingDetection  jammingDetection = null;
    protected double antiJammingPower = 0.3;
    protected double antiJammingPowerStepSize = 0.01;

    public WallTracker(HardwareWallTracker wallTracker,
                       DcMotor leftW,
                       DcMotor rightW,
                       int bufferS){
        wallTrackerHW = wallTracker;
        leftWheel = leftW;
        rightWheel = rightW;
        bufferSize = bufferS;
        distanceBuffer = new double[bufferSize];
        powerBuffer = new double[bufferSize];
        jammingDetection = new JammingDetection (1000L);
        skewPowerGain = 1.0/180;  // adjust turn sensitivity
    }

    public void setTargetWallDistance (double d) {
        targetDistance = d;
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        wallTrackerHW.sonicRange.enableLed(true);

        state =0;

        calibrate();

        minTurnPower = 0.02;
        maxTurnPower = 0.35;
        skewPowerGain = 1.0/180;
        skewTolerance = 0;

    }

    /**
     *
     * @param startState, The initial state of wall follower
     */
    @Override
    public void start(int startState) {

        state = startState;
        jammingDetection.reset();

    }

    public void loop (double power, double powerDelta, double gain) {

        // jamming detection
        if (jammingDetection.isJammed(Math.min(leftWheel.getCurrentPosition(),
                rightWheel.getCurrentPosition()))) {
            report("Wall Tracker", " Jammed");
            antiJammingPower *= -1;
            leftWheel.setPower(antiJammingPower);
            rightWheel.setPower(-antiJammingPower);
            return ;
        }

        // normal activity
        switch (state) {
            case 0:
                // detect wall
                state = detectWall(power, powerDelta); // left turn
                report("Wall Tracker", "state = DETECT");
                break;
            case 1:
                // follow wall
                state = followWall(power, targetDistance, gain);
                report("Wal Tracker", "state = FOLLOW");
                break;
            case 2:
                // search wall
                state = searchWall(power);
                report("Wall Tracker", "state = SEARCH");
                break;
            default:
                break;
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        leftWheel.setPower(0.0);
        rightWheel.setPower(0.0);
        wallTrackerHW.sonicRange.enableLed(false);
    }

    public double readDistance() {

        double d =  wallTrackerHW.sonicRange.getDistance(DistanceUnit.CM);
        distanceBuffer[bufferIndex] =d;
        powerBuffer[bufferIndex] = lastDirection;
        bufferIndex++;
        if (bufferIndex >= bufferSize) {
            bufferIndex = 0;
        }
        return d;
    }

    public double getHistoryDistanceAverage() {
        double bl = 0;
        double maxV = -1;
        double minV = 100000;

        for (int i = 0; i < bufferSize; i++) {
            int index = Math.max(0,bufferIndex -i);
            double v = distanceBuffer[index];
            if ( v > maxV) {
                maxV = v;
            }
            if ( v < minV) {
                minV = v;
            }
            bl += distanceBuffer[index];
        }

        if (bufferSize >=5) {
            // remove max and min to avoid noise
            return (bl  - minV - maxV) / (bufferSize - 2);
        }

        return bl / bufferSize;
    }

    public double getHistoryPowerAverage() {
        double bl = 0;

        for (int i = 0; i < bufferSize; i++) {
            int index = Math.max(0,bufferIndex -i);
            bl += powerBuffer[index];
        }

        return bl / bufferSize;
    }

    public void calibrate() {
        for (int i = 0; i < bufferSize; i++) {
            readDistance();
        }
    }

    public int detectWall (double power, double powerDelta) {

        readDistance();

        // go straight until it find the wall
        double l = getHistoryDistanceAverage();

        report("Wall Distance", df3.format(l) );

        if ( l < targetDistance) {
            leftWheel.setPower(0);
            rightWheel.setPower(0);
            return 1;
        }
        else {
            leftWheel.setPower(power+powerDelta);
            rightWheel.setPower(power-powerDelta);
        }

        return 0;

    }

    /**
     *
     * @param power , main power
     * @param targetDis, target distance
     * @param gain, if > 0, wall on the right. < 0 , wall on the left. Weight value as well
     * @return the state of wall tracker
     */
    public int followWall(double power, double targetDis, double gain) {

        if (power == 0) return 1;

        readDistance();
        double avgD = getHistoryDistanceAverage();

        report("Wall Distance  :", df3.format(avgD) );
        report("target Distance:", df3.format(targetDistance) );

        if (avgD > lostDistance) {
            leftWheel.setPower(0);
            rightWheel.setPower(0);
            return 2; // search for wall
        }

        double delta = (avgD - targetDis) * power/Math.abs(power);
        lastDirection = computeTurnPower(delta)*gain;
        //lastDirection = VortexUtils.lookUpTableFunc(delta, dist2PowerLUT)*gain;
        double left  = Range.clip(power - lastDirection, -1, 1);
        double right = Range.clip(power + lastDirection, -1, 1);
        leftWheel.setPower(left);
        rightWheel.setPower(right);
        report("Left Wheel power:", Double.toString(left));
        report("Right Wheel power:", Double.toString(right));
        report("Last Direction", Double.toString(lastDirection));

        return 1;

    }

    public int searchWall (double power) {

        readDistance();
        double avgD = getHistoryDistanceAverage();
        if (avgD > lostDistance) {
            // find  (distance, power) in history, reverse the process
            double avgP = getHistoryPowerAverage();
            double left  = Range.clip(power - avgP, -1, 1);
            double right = Range.clip(power + avgP, -1, 1);
            leftWheel.setPower(left);
            rightWheel.setPower(right);
            return 2;
        }
        else {
            // back to follow state
            leftWheel.setPower(0);
            rightWheel.setPower(0);
            return 1;
        }
    }
}
