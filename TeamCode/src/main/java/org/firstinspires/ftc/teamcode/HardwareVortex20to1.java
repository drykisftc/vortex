package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This is NOT an OpMode
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot, using Matrix Hardware.
 * See PushbotTeleopTank_Iterative for a usage examples.
 *
 * This is coded as an Extension of HardwarePushbot to illustrate that the only additional
 * action REQUIRED for a MATRIX controller is enabling the Servos.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Matrix Controller has been assigned the name:  "matrix controller"
 *
 * Motor channel:  Left  drive motor:        "left motor"
 * Motor channel:  Right drive motor:        "right motor"
 * Motor channel:  Manipulator drive motor:  "arm motor"
 * Servo channel:  Servo to open left claw:  "left claw"
 * Servo channel:  Servo to open right claw: "right claw"
 *
 * In addition, the Matrix Controller has been assigned the name:  "matrix controller"
 */
public class HardwareVortex20to1
{

    HardwareMap hwMap           =  null;

    // DC Motors
    public DcMotor motorLeftArm = null;
    public DcMotor motorRightArm = null;
    public DcMotor motorLeftWheel = null;
    public DcMotor motorRightWheel =null;
    public DcMotor motorLeftHand = null;
    public DcMotor motorRightHand = null;

    // Servos

    // Sensors

    // Camera

    /* Constructor */
    public HardwareVortex20to1(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        motorLeftWheel   = hwMap.dcMotor.get("leftWheel");
        motorRightWheel  = hwMap.dcMotor.get("rightWheel");
        motorLeftWheel.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorRightWheel.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        motorLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeftArm   = hwMap.dcMotor.get("leftArm");
        motorRightArm  = hwMap.dcMotor.get("rightArm");
        motorLeftArm.setDirection(DcMotor.Direction.FORWARD);
        motorRightArm.setDirection(DcMotor.Direction.REVERSE);
        motorLeftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeftHand   = hwMap.dcMotor.get("leftHand");
        motorRightHand  = hwMap.dcMotor.get("rightHand");
        motorLeftHand.setDirection(DcMotor.Direction.REVERSE);
        motorRightHand.setDirection(DcMotor.Direction.FORWARD);
        motorLeftHand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightHand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void stop() {
        motorLeftWheel.setPower(0.0);
        motorRightWheel.setPower(0.0);
        motorLeftArm.setPower(0.0);
        motorRightArm.setPower(0.0);
        motorLeftHand.setPower(0.0);
        motorRightHand.setPower(0.0);

        motorLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
