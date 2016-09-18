package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.ArmableUsbDevice;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.DecimalFormat;

public class LineTracker {

    OpticalDistanceSensor leftODS = null;
    OpticalDistanceSensor rightODS = null;
    OpticalDistanceSensor middleODS = null;

    DcMotor leftWheel = null;
    DcMotor rightWheel = null;

    // state machine
    public int state = 0;

    int bufferSize = 10;
    double[] leftBuffer = null;
    double[] rightBuffer = null;
    double[] middleBuffer = null;
    int bufferIndex = 0;
    double baselineODS = 0.0d;
    double baselineSensitivity = 2.0;
    double lastDirection = 0.0d;

    double[] ods2PowerLUT = {0.0f, 0.25f, 0.33f, 0.41f,
            0.49f, 0.56f, 0.63f, 0.70f, 0.77f, 0.84f,
            0.91f, 0.96f, 1.0f};

    double[] ods2PowerSearchLUT = {0.0f, 0.25f, 0.58f, 0.91f, 1.0f};

    Telemetry reporter = null;

    private static DecimalFormat df3 = new DecimalFormat(".###");


    public LineTracker(OpticalDistanceSensor leftO,
                       OpticalDistanceSensor rightO,
                       OpticalDistanceSensor middleO,
                       DcMotor leftW,
                       DcMotor rightW,
                       int bufferS){
        leftODS = leftO;
        rightODS = rightO;
        middleODS = middleO;
        leftWheel = leftW;
        rightWheel = rightW;
        bufferSize = bufferS;
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init() {

        leftBuffer = new double[bufferSize];
        rightBuffer = new double[bufferSize];
        middleBuffer = new double[bufferSize];

        leftODS.enableLed(true);
        rightODS.enableLed(true);
        middleODS.enableLed(true);

        state =0;

        calibrate();

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        // compute baseline brightness
        baselineODS = Math.min(0.5, getBaselineODS() * baselineSensitivity);
        state =0;
    }

    public void loop () {

        switch (state) {
            case 0:
                // detect line
                state = detectLine(0.1);
                reporter.addData("Line Tracker", "state = DETECT");
                break;
            case 1:
                // follow line
                state = followLine(0.1);
                reporter.addData("Line Tracker", "state = FOLLOW");
                break;
            case 2:
                // search for line
                state = searchLine(0.1);
                reporter.addData("Line Tracker", "state = SEARCH");
                break;
            default:
                break;
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    public void stop() {
        leftWheel.setPower(0.0);
        rightWheel.setPower(0.0);
        leftODS.enableLed(false);
        rightODS.enableLed(false);
        middleODS.enableLed(false);
    }

    public void readODS() {

        leftBuffer[bufferIndex] = leftODS.getLightDetected();
        rightBuffer[bufferIndex] = rightODS.getLightDetected();
        middleBuffer[bufferIndex] = middleODS.getLightDetected();
        bufferIndex++;
        if (bufferIndex >= bufferSize) {
            bufferIndex = 0;
        }
    }

    public double getBaselineODS() {
        double bl = 0;
        for (int i = 0; i < bufferSize; i++) {
            bl += leftBuffer[i];
            bl += rightBuffer[i];
            bl += middleBuffer[i];
        }

        return bl / 3.0 / bufferSize;
    }

    public void calibrate() {
        for (int i = 0; i < bufferSize; i++) {
            readODS();
        }
        baselineODS = Math.min(0.5, getBaselineODS() * baselineSensitivity);
    }

    public int detectLine(double power) {
        int vote = 0;

        double l = leftODS.getLightDetected();
        double r = rightODS.getLightDetected();
        double m = middleODS.getLightDetected();
        if (l > baselineODS) vote++;
        if (r > baselineODS) vote++;
        if (m > baselineODS) vote++;

        if (reporter != null) {
            reporter.addData("ODS", df3.format(l) + ": " + df3.format(m) + ": " + df3.format(r));
            reporter.addData("ODS Baseline", df3.format(baselineODS));
        }

        if (vote >= 2) {
            // stop
            leftWheel.setPower(0.0d);
            rightWheel.setPower(0.0d);



            return 1;
        } else {
            leftWheel.setPower(power);
            rightWheel.setPower(power);
            return 0;
        }

    }

    public int followLine(double power) {

        if (power == 0.0) {
            leftWheel.setPower(power);
            rightWheel.setPower(power);
            return 1;
        }

        double l = leftODS.getLightDetected();
        double r = rightODS.getLightDetected();

        if (reporter != null) {
            reporter.addData("ODS", df3.format(l) + ": " + df3.format(r));
            reporter.addData("ODS Baseline", df3.format(baselineODS));
        }

        if ( l <= baselineODS && r <= baselineODS) {
            double m = middleODS.getLightDetected();
            if (m < baselineODS){
                if (reporter != null) {
                    reporter.addData("ODS", df3.format(l) + ": " + df3.format(m) + ": " + df3.format(r));
                    reporter.addData("ODS Baseline", df3.format(baselineODS));
                }
                leftWheel.setPower(0.0);
                rightWheel.setPower(0.0);
                return 2;
            }
        }
        // ensure that it works when it move backward as well
        double delta = Range.clip((l - r)* power / Math.abs(power), -1, 1);
        lastDirection = ResQUtils.lookUpTableFunc(delta, ods2PowerLUT);
        double left  = Range.clip(power + lastDirection, -1, 1);
        double right = Range.clip(power - lastDirection, -1, 1);
        leftWheel.setPower(left);
        rightWheel.setPower(right);

        return 1;
    }

    public int searchLine (double power) {

        if (power == 0.0) {
            leftWheel.setPower(power);
            rightWheel.setPower(power);
            return 2;
        }

        double l = leftODS.getLightDetected();
        double r = rightODS.getLightDetected();
        double m = middleODS.getLightDetected();

        if (reporter != null) {
            reporter.addData("ODS", df3.format(l) + ": " + df3.format(m) + ": " + df3.format(r));
            reporter.addData("ODS Baseline", df3.format(baselineODS));
        }

        int newState = 2;

        if (l <= baselineODS &&
                m <= baselineODS &&
                r <= baselineODS) {

            // keep same search direction
            if (Math.abs(lastDirection) >=0) {
                lastDirection= Math.max(Math.abs(power), lastDirection);
            }
            else {
                lastDirection= Math.min(-Math.abs(power), lastDirection);
            }
        } else {
            newState = 1;
            // ensure that it works when it move backward as well
            double delta = Range.clip((l - r) * power / Math.abs(power), -1, 1);
            lastDirection = ResQUtils.lookUpTableFunc(delta, ods2PowerSearchLUT);
        }

        if ( lastDirection >=0 ) {
            double left = Range.clip(lastDirection*0.6, -1, 1);
            double right = Range.clip(-lastDirection*1.6, -1, 1);
            leftWheel.setPower(left);
            rightWheel.setPower(right);
        }
        else {
            double left = Range.clip(lastDirection*1.6, -1, 1);
            double right = Range.clip(-lastDirection*0.6, -1, 1);
            leftWheel.setPower(left);
            rightWheel.setPower(right);
        }


        return newState;
    }

    public void setReporter (Telemetry t){
        reporter = t;
    }

}
