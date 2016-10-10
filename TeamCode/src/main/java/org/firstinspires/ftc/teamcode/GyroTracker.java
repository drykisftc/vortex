package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.DecimalFormat;

public class GyroTracker {

    ModernRoboticsI2cGyro gyro = null;

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

    public void setWallDistance (double d) {
        targetDistance = d;
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init() {

        distanceBuffer = new double[bufferSize];
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

    public void loop () {


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
