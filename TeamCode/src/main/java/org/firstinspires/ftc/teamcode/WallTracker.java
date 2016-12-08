package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.text.DecimalFormat;

public class WallTracker extends RobotExecutor {

    HardwareWallTracker wallTrackerHW = null;
    ModernRoboticsI2cRangeSensor rangeSensor = null;

    DcMotor leftWheel = null;
    DcMotor rightWheel = null;

    // state machine
    public int state = 0;

    private int bufferSize = 10;
    private double[] distanceBuffer = null;
    private double[] powerBuffer = null;
    private int bufferIndex = 0;

    private double targetDistance = 5.0; // unit cm
    private double lastDirection = 0.0d;
    private double lostDistance = 100;

    double[] dist2PowerLUT = {0.0f, 0.05f, 0.15f, 0.18f, 0.20f,
            0.22f, 0.24f, 0.26f, 0.28f, 0.30f, 0.32f, 0.34f, 0.36f,
            0.38f, 0.42f, 0.46f, 0.50f, 0.54f, 0.58f, 0.62f, 0.66f,
            0.70f, 0.74f, 0.78f, 0.82f, 0.86f, 0.90f, 0.94f, 0.98f, 1.00f};

    private static DecimalFormat df3 = new DecimalFormat(".###");

    public WallTracker(HardwareWallTracker wallTracker,
                       DcMotor leftW,
                       DcMotor rightW,
                       int bufferS){
        rangeSensor = wallTracker.sonicRange;
        wallTrackerHW = wallTracker;
        leftWheel = leftW;
        rightWheel = rightW;
        bufferSize = bufferS;
    }

    public void setWallDistance (double d) {
        targetDistance = d;
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        distanceBuffer = new double[bufferSize];
        powerBuffer = new double[bufferSize];

        rangeSensor.enableLed(true);

        state =0;

        calibrate();

    }

    /**
     *
     * @param startState, The initial state of wall follower
     */
    @Override
    public void start(int startState) {

        state = startState;
    }

    public void loop (double power, double powerDelta, double direction) {

        switch (state) {
            case 0:
                // detect wall
                state = detectWall(power);
                report("Wall Tracker", "state = DETECT");
                break;
            case 1:
                // detect wall
                state = followWall(power, powerDelta,targetDistance, direction);
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
        rangeSensor.enableLed(false);
    }

    public double readDistance() {

        double d =  rangeSensor.getDistance(DistanceUnit.CM);
        distanceBuffer[bufferIndex] =d;
        powerBuffer[bufferIndex] = lastDirection;
        bufferIndex++;
        if (bufferIndex >= bufferSize) {
            bufferIndex = 0;
        }
        return d;
    }

    public double getHistoryDistanceAverage(int range) {
        double bl = 0;

        for (int i = 0; i < range; i++) {
            int index = Math.max(0,bufferIndex -i);
            bl += distanceBuffer[index];
        }

        return bl / bufferSize;
    }

    public double getHistoryPowerAverage(int range) {
        double bl = 0;

        for (int i = 0; i < range; i++) {
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

    public int detectWall (double power) {

        // go straight until it find the wall
        double l = getHistoryDistanceAverage(4);

        report("Wall Distance", df3.format(l) );

        if ( l < targetDistance) {
            leftWheel.setPower(0);
            rightWheel.setPower(0);
        }
        else {
            leftWheel.setPower(power);
            rightWheel.setPower(power);
        }

        return 0;

    }

    /**
     *
     * @param power , main power
     * @param powerDelta, correction power
     * @param distance, target distance
     * @param signValue, if > 0, wall on the right. < 0 , wall on the left. Weight value as well
     * @return the state of wall tracker
     */
    public int followWall(double power, double powerDelta, double distance, double signValue) {

        double l = readDistance();

        report("Wall Distance", df3.format(l) );

        double avgD = getHistoryDistanceAverage(4);
        if (avgD > lostDistance) {
            leftWheel.setPower(0);
            rightWheel.setPower(0);
            return 2;
        }

        double delta = Range.clip((l - distance)/lostDistance * power / Math.abs(power), -1, 1);
        lastDirection = VortexUtils.lookUpTableFunc(delta, dist2PowerLUT)*signValue;
        double left  = Range.clip(power + lastDirection + powerDelta, -1, 1);
        double right = Range.clip(power - lastDirection + powerDelta, -1, 1);
        leftWheel.setPower(left);
        rightWheel.setPower(right);

        return 1;

    }

    public int searchWall (double power) {

        readDistance();
        double avgD = getHistoryDistanceAverage(4);
        if (avgD > lostDistance) {
            // find  (distance, power) in history, reverse the process
            double avgP = getHistoryPowerAverage(4);
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
