package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by hfu on 5/11/17.
 */

public class EncoderTracker extends Tracker {

    private DcMotor leftWheel = null;
    private DcMotor rightWheel = null;
    private double axleLength = 145.3;
    private int leftLandmark = 0;
    private int rightLandmark =0;
    private int state = 0;
    private double tolerance = 5.0;

    public EncoderTracker (DcMotor l, DcMotor r, double al) {
        leftWheel = l;
        rightWheel = r;
        axleLength = al;
    }

    public void init(double t) {
        state = 1;
        tolerance = t;
    }

    public void stop () {
        state = 0;
    }

    public int turn (double degree, double power) {
        if (degree ==0.0) return 0;
        switch (state) {
            case 1:
                // measure current position
                leftLandmark = leftWheel.getCurrentPosition();
                rightLandmark = rightWheel.getCurrentPosition();
                turnByPower(degree, power);
                state = 2;
                break;
            case 2:
                // measure turn angle and adjust
                int leftDistance = leftWheel.getCurrentPosition() - leftLandmark;
                int rightDistance = rightWheel.getCurrentPosition() - rightLandmark;
                int l = Math.abs(leftDistance);
                int r = Math.abs(rightDistance);
                int longerMove = leftDistance;
                int sign = -1;
                if (l < r) {
                    longerMove = rightDistance;
                    sign = 1;
                }
                double radius = Math.abs(axleLength*longerMove/(l+r));
                double turnedDegree = -180.0 * sign * longerMove / (Math.PI * radius);
                if (Math.abs(degree - turnedDegree) <= tolerance) {
                    state = 0;
                    leftWheel.setPower(0.0);
                    rightWheel.setPower(0.0);
                } else {
                    turnByPower(degree, power);
                }
                break;
            default:
                leftWheel.setPower(0.0);
                rightWheel.setPower(0.0);
                break;
        }
        return state;
    }

    private void turnByPower (double degree, double power) {
        if (degree > 0) {
            leftWheel.setPower(-1 * power);
            rightWheel.setPower(power);
        } else {
            leftWheel.setPower(power);
            rightWheel.setPower(-1 * power);
        }
    }
}
