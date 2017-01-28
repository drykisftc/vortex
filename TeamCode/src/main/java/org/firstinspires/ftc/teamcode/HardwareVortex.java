package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

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
public class HardwareVortex extends HardwareBase
{
    // DC Motors
    public DcMotor motorLeftArm = null;
    public DcMotor motorRightArm = null;
    public DcMotor motorLeftWheel = null;
    public DcMotor motorRightWheel = null;
    public DcMotor motorLeftHand = null;
    public DcMotor motorRightHand = null;

    // Servos
    public Servo servoCock = null;

    public CRServo servoLeftScooper = null;
    public CRServo servoRightScooper = null;

    // Sensors
    public TouchSensor armStopMin = null;
    public TouchSensor armStopMax = null;

    // Camera

    /* Constructor */
    public HardwareVortex(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        super.init(ahwMap);

        servoCock = hwMap.servo.get("cock");
        servoLeftScooper = hwMap.crservo.get("leftScooper");
        servoRightScooper = hwMap.crservo.get("rightScooper");
        servoLeftScooper.setPower(0);
        servoRightScooper.setPower(0);

        armStopMin = hwMap.touchSensor.get("armStopMin");
        armStopMax = hwMap.touchSensor.get("armStopMax");

        motorLeftWheel   = hwMap.dcMotor.get("leftWheel");
        motorRightWheel  = hwMap.dcMotor.get("rightWheel");
        motorLeftWheel.setDirection(DcMotor.Direction.FORWARD);  // 40 to 1 andymark motor
        motorRightWheel.setDirection(DcMotor.Direction.REVERSE); // 40 to 1 andymark motor
        motorLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftWheel.setPower(0.0);
        motorRightWheel.setPower(0.0);

        motorLeftArm   = hwMap.dcMotor.get("leftArm");
        motorRightArm  = hwMap.dcMotor.get("rightArm");
        motorLeftArm.setDirection(DcMotor.Direction.REVERSE);
        motorRightArm.setDirection(DcMotor.Direction.REVERSE);
        motorLeftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftArm.setPower(0.0);
        motorRightArm.setPower(0.0);

        motorLeftHand   = hwMap.dcMotor.get("leftHand");
        motorRightHand  = hwMap.dcMotor.get("rightHand");
        motorLeftHand.setDirection(DcMotor.Direction.FORWARD);
        motorRightHand.setDirection(DcMotor.Direction.FORWARD);
        motorLeftHand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightHand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
