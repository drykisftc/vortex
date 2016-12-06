/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.VortexTeleOp.LeftArmState.FIRE;
import static org.firstinspires.ftc.teamcode.VortexTeleOp.LeftArmState.LOAD;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwareVortex class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp: A pro", group="Run")
class VortexTeleOp extends OpMode{

    /* Declare OpMode members. */
    protected HardwareVortex robot = null;
    protected HardwareWallTracker wallTrackerHW = null;
    protected ParticleShooter particleShooter = null;

    boolean boolLeftArmEnable = true;
    boolean boolRightArmEnable = false;

    private int leftArmHomePosition = 0;
    private int rightArmHomePosition = 0;

    private final int leftArmHomeParkingOffset = 80;
    private final int leftArmLoadPositionOffset = 650;
    protected int leftArmMovePositionOffset = 1050;
    private final int leftArmSnapPositionOffset = 50;
    private final int leftArmFirePositionOffset = 4610;
    private final int leftArmMaxOffset = 4610;
    private int leftArmFiringSafeZoneOffset = 3500;

    private int leftArmHomeParkingPostion = leftArmHomeParkingOffset;
    private int leftArmLoadPosition = leftArmLoadPositionOffset;
    protected int leftArmMovePosition = leftArmMovePositionOffset;
    private int leftArmSnapPosition= leftArmSnapPositionOffset;
    private int leftArmFirePosition = leftArmFirePositionOffset;
    private int leftArmFiringSafeZone = leftArmFiringSafeZoneOffset;
    private int leftArmPeakPosition = leftArmMaxOffset/2;
    private int leftArmMaxRange = leftArmMaxOffset;

    private double leftArmJoystickDeadZone = 0.05;
    private double leftArmHoldPower = 0.2;
    protected double leftArmAutoMovePower = 0.4;
    private double leftArmAutoSlowMovePower = 0.1;
    private double leftArmHomingMovePower = -0.2;
    private long leftArmHomingTimestamp =0;
    private long leftArmHomingTime =5000;

    private int leftArmMinLimitSwitchOnCount =0;
    private int leftArmMaxLimitSwitchOnCount =0;
    private int leftArmLimitSwitchCountThreshold = 8;

    private double leftHandPowerDefaultAttenuate = 0.5;

    enum LeftArmState {
        HOME,
        LOAD,
        FIRE,
        MANUAL
    }

    private LeftArmState leftArmState = LeftArmState.HOME;

    boolean isArmHoldingPositionSet = false;

    int armTargetPosition = 0;

    double [] wheelPowerLUT = {0.0f, 0.05f, 0.12f, 0.14f, 0.16f, 0.18f, 0.20f,
            0.22f, 0.24f, 0.26f, 0.28f, 0.30f, 0.32f, 0.34f, 0.36f,
            0.38f, 0.40f, 0.42f, 0.44f, 0.46f, 0.48f, 0.50f, 0.54f, 0.58f, 0.6f,
            0.62f, 0.66f, 0.70f, 0.74f, 0.78f, 0.82f, 0.86f, 0.90f, 0.94f, 0.98f, 1.00f};

    double [] leftArmPowerLUT = { 0.0f, 0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f, 0.07f,
            0.08f, 0.09f, 0.10f, 0.11f, 0.12f, 0.13f, 0.14f, 0.15f,
            0.16f, 0.17f, 0.18f, 0.19f, 0.20f, 0.21f, 0.22f, 0.23f,
            0.24f, 0.25f, 0.26f, 0.27f, 0.28f, 0.29f, 0.30f, 0.31f,
            0.32f, 0.33f, 0.34f, 0.35f, 0.36f, 0.37f, 0.38f, 0.39f,
            0.40f, 0.45f};

    double [] armDistanceLUT = { 0.0f, 0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f, 0.07f,
            0.08f, 0.09f, 0.10f, 0.11f, 0.12f, 0.13f, 0.14f, 0.15f,
            0.17f, 0.18f, 0.19f, 0.20f, 0.21f, 0.22f, 0.23f, 0.24f,
            0.25f, 0.26f, 0.27f, 0.28f, 0.29f, 0.30f, 0.31f, 0.32f,
            0.33f, 0.34f, 0.35f, 0.36f, 0.37f, 0.38f, 0.39f, 0.40f,
            0.41f, 0.42f, 0.43f, 0.44f, 0.45f, 0.46f, 0.47f, 0.48f,
            0.49f, 0.50f, 0.51f, 0.52f, 0.53f, 0.54f, 0.55f, 0.56f,
            0.60f, 0.65f, 0.70f, 0.75f, 0.80f, 0.85f, 0.90f, 0.95f, 1.0f };

    double [] armSpeedLUT = { 0.0f, 0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f, 0.07f,
            0.08f, 0.09f, 0.10f, 0.11f, 0.12f, 0.13f, 0.14f, 0.15f,
            0.17f, 0.18f, 0.19f, 0.20f, 0.21f, 0.22f, 0.23f, 0.24f,
            0.25f, 0.26f, 0.27f, 0.28f, 0.29f, 0.30f, 0.31f, 0.32f,
            0.33f, 0.34f, 0.35f, 0.36f, 0.37f, 0.38f, 0.39f, 0.40f,
            0.41f, 0.42f, 0.43f, 0.44f, 0.45f, 0.46f, 0.47f, 0.48f,
            0.49f, 0.50f, 0.51f, 0.52f, 0.53f, 0.54f, 0.55f, 0.56f,
            0.60f, 0.65f, 0.70f, 0.75f, 0.80f, 0.85f, 0.90f, 0.95f, 1.0f };



    // left arm control information
    HardwareBeaconArm leftBeaconArm = null;
    private boolean leftLoopTrue = false;
    private double leftUpHomePosition = 0.90;
    private double leftUpStepSize = -0.015;
    private double leftLowHomePosition = 0.95;
    private double leftLowStepSize = -0.05;
    /* Important: use the core device discovery tool to set color sensor address to 0x40
    Then, use the 7 bit version of it 0x20
     */
    private int    leftBeaconColorSensorAddr = 0x20;

    // right arm control information
    HardwareBeaconArm rightBeaconArm = null;
    private boolean rightLoopTrue = false;
    private double rightUpHomePosition = 0.1;
    private double rightUpStepSize = 0.015;
    private double rightLowHomePosition = 0.02;
    private double rightLowStepSize = 0.05;
    /* Important: use the core device discovery tool to set color sensor address to 0x48
    Then, use the 7 bit version of it 0x24
     */
    private int    rightBeaconColorSensorAddr = 0x24; //

    // scooper control
    double leftScooperStop = 0.0;
    double leftScooperGo = -1.0;
    double rightScooperStop = 0.0;
    double rightScooperGo = 1.0;

    protected int leftArmCurrentPosition = 0;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        robot = new HardwareVortex();

        robot.init(hardwareMap);

        // beacon arm
        initBeaconArms();

        // hands
        particleShooter = new ParticleShooter(robot.motorLeftArm,
                robot.motorLeftHand, robot.servoCock, robot.armStopMax);
        particleShooter.init();
        particleShooter.setReporter(telemetry);
        particleShooter.armFiringPosition = leftArmFirePosition;
        particleShooter.armFiringSafeZone = leftArmFiringSafeZone;
        particleShooter.relax();
        particleShooter.start(0);
        particleShooter.resetArmJammed();

        // wall tracker
        wallTrackerHW = new HardwareWallTracker();
        wallTrackerHW.init(hardwareMap);
        wallTrackerHW.park();

        // arms
        leftArmMinLimitSwitchOnCount =0;
        leftArmHomingTimestamp = System.currentTimeMillis();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("TeleOp", "Hello Vortex");    //
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        // homing the left arm. If the touch sensor is on, turn off arm power
        if ( (!particleShooter.isArmJammed(robot.motorLeftArm.getCurrentPosition()))
                && leftArmMinLimitSwitchOnCount < leftArmLimitSwitchCountThreshold
                && System.currentTimeMillis() - leftArmHomingTimestamp < leftArmHomingTime) {
            robot.motorLeftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorLeftArm.setPower(leftArmHomingMovePower);
            telemetry.addData("TeleOp", "Calibrating Arm....");
        } else {
            robot.motorLeftArm.setPower(0.0);
            telemetry.addData("TeleOp", "Ready to start");
        }

        if (robot.armStopMin.isPressed()) {
            leftArmMinLimitSwitchOnCount++;
        } else {
            leftArmMinLimitSwitchOnCount =0;
        }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        // wheels
        robot.motorLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLeftWheel.setPower(0.0);
        robot.motorRightWheel.setPower(0.0);

        // arms
        robot.motorLeftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLeftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLeftArm.setPower(0.0);
        robot.motorRightArm.setPower(0.0);
        leftArmCurrentPosition = robot.motorLeftArm.getCurrentPosition();
        leftArmHomePosition = leftArmCurrentPosition;
        telemetry.addData("left arm home",  "%2d", leftArmHomePosition);
        rightArmHomePosition = robot.motorRightArm.getCurrentPosition();
        leftArmState = LeftArmState.HOME;

        leftArmHomeParkingPostion = leftArmHomePosition + leftArmHomeParkingOffset;
        leftArmMovePosition = leftArmHomePosition+leftArmMovePositionOffset;
        leftArmSnapPosition= leftArmHomePosition+leftArmSnapPositionOffset;
        leftArmFirePosition = leftArmHomePosition+ leftArmFirePositionOffset;
        leftArmFiringSafeZone = leftArmHomePosition+ leftArmFiringSafeZoneOffset;
        leftArmPeakPosition = leftArmHomePosition + leftArmMaxOffset/2;
        leftArmMaxRange = leftArmHomePosition + leftArmMaxOffset;

        leftArmMinLimitSwitchOnCount = 0;
        leftArmMaxLimitSwitchOnCount =0;

        particleShooter.armFiringPosition = leftArmFirePosition;
        particleShooter.armFiringSafeZone = leftArmFiringSafeZone;
        particleShooter.resetArmJammed();

        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // query sensors
        if (robot.armStopMin.isPressed()) {
            leftArmMinLimitSwitchOnCount ++;
        } else {
            leftArmMinLimitSwitchOnCount = 0;
        }

        if (robot.armStopMax.isPressed()) {
            leftArmMaxLimitSwitchOnCount ++;
        } else {
            leftArmMaxLimitSwitchOnCount = 0;
        }

        leftArmCurrentPosition = robot.motorLeftArm.getCurrentPosition();

        wheelControl();
        leftArmControl();
        leftHandControl();
        elevatorControl();
        beaconArmControl();
        scooperControl();
        telemetry.update();
    }

    public void wheelControl() {
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        float throttle = -gamepad1.left_stick_y;
        float direction = gamepad1.left_stick_x;
        float right = throttle - direction;
        float left = throttle + direction;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        robot.motorLeftWheel.setPower(left);
        robot.motorRightWheel.setPower(right);

        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
    }

    public void leftArmControl() {

        // buttons
        if (gamepad1.a) {
            // go to home position
            particleShooter.resetArmJammed();
            leftArmState = LeftArmState.HOME;
        } else if (gamepad1.b) {
            // go to load position
            particleShooter.resetArmJammed();
            leftArmState = LeftArmState.LOAD;
        } else if (gamepad1.y) {
            // go to shoot position
            particleShooter.resetArmJammed();
            leftArmState = FIRE;
        }

        // move arms by power
        float throttle = gamepad1.right_stick_y;

        if (Math.abs(throttle) >= leftArmJoystickDeadZone) {
            // set to manual mode
            leftArmState = LeftArmState.MANUAL;
            isArmHoldingPositionSet = false;
            particleShooter.resetArmJammed();

            // get joystick position
            double power = Range.clip(VortexUtils.lookUpTableFunc(throttle, leftArmPowerLUT), -1, 1);
            telemetry.addData("arm power", "%.2f", power);

            // move arms
            robot.motorLeftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("left arm pos", "%6d vs %6d", leftArmCurrentPosition, leftArmMaxRange);

            // button x allow arm moving to home, just in case the arm loses encoder home position
            // giving a chance to reset home position
            if ((!gamepad1.x)
                    && leftArmMinLimitSwitchOnCount > leftArmLimitSwitchCountThreshold
                    && power < 0.0) {
                telemetry.addData("left arm min exceed", "STOP!!!!!!! %d", leftArmMinLimitSwitchOnCount);
                stopLeftArm();
            } else if ((!gamepad1.x)
                    && (leftArmCurrentPosition > leftArmMaxRange
                    || leftArmMaxLimitSwitchOnCount > leftArmLimitSwitchCountThreshold)
                    && power > 0.0) {
                telemetry.addData("left arm max exceeded", "STOP!!!!!!!!!!!!!! %d", leftArmMaxLimitSwitchOnCount);
                stopLeftArm();
            } else {
                robot.motorLeftArm.setPower(power);
                robot.motorRightArm.setPower(0.0);
            }
        }
        else {
            switch (leftArmState) {
                case HOME: {
                    // go to home position
                    if (leftArmCurrentPosition <= leftArmHomeParkingPostion) {
                        robot.motorLeftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorLeftArm.setPower(0.0);
                    } else if (leftArmCurrentPosition < leftArmPeakPosition) {
                        VortexUtils.moveMotorByEncoder(robot.motorLeftArm, leftArmHomeParkingPostion, leftArmAutoSlowMovePower);
                    } else {
                        VortexUtils.moveMotorByEncoder(robot.motorLeftArm, leftArmHomeParkingPostion, leftArmAutoMovePower);
                    }
                }
                    break;
                case LOAD: {
                    // if trigger snap, move between snap position and load position
                    double snapTrigger = gamepad1.left_trigger;
                    if (snapTrigger > leftArmJoystickDeadZone) {
                        int target = leftArmLoadPosition
                                - (int) ((leftArmLoadPosition - leftArmHomePosition) * snapTrigger);
                        VortexUtils.moveMotorByEncoder(robot.motorLeftArm, target, leftArmAutoMovePower);
                    } else {
                        VortexUtils.moveMotorByEncoder(robot.motorLeftArm, leftArmMovePosition, leftArmAutoMovePower);
                    }
                }
                    break;
                case FIRE:
                    // move to fire position
                {
                    if (particleShooter.isArmJammed(leftArmCurrentPosition))
                    {
                        telemetry.addData("left arm jammed", "STOP!!!!!!!!!!!!!! %d", leftArmCurrentPosition);
                        stopLeftArm();
                    } else if (leftArmMaxLimitSwitchOnCount > leftArmLimitSwitchCountThreshold) {
                        telemetry.addData("left arm max exceeded", "STOP!!!!!!!!!!!!!! %d", leftArmMaxLimitSwitchOnCount);
                        stopLeftArm();
                    } else {
                        VortexUtils.moveMotorByEncoder(robot.motorLeftArm, leftArmFirePosition, leftArmAutoMovePower);
                    }
                }
                    break;
                case MANUAL:
                default:
                    // hold current position
                    if (isArmHoldingPositionSet) {
                        VortexUtils.moveMotorByEncoder(robot.motorLeftArm, armTargetPosition, leftArmHoldPower);
                    } else {
                        // set the target position to limits.
                        if ( leftArmCurrentPosition >= leftArmHomeParkingPostion) {
                            armTargetPosition = Math.max(leftArmHomeParkingPostion, leftArmCurrentPosition);
                            armTargetPosition = Math.min(leftArmMaxRange, armTargetPosition);
                            isArmHoldingPositionSet = true;
                        }
                        // stop motor to prevent it overshoot the limits. Power will enable in next loop call.
                        robot.motorLeftArm.setPower(0.0);
                    }
            }
        }
    }

    public void leftHandControl() {

        if (gamepad1.right_bumper || gamepad2.left_trigger > 0.7) {
            particleShooter.calibrateHandByBall();
        } else if ( (leftArmState == FIRE
                && gamepad1.left_bumper) || gamepad2.right_trigger > 0.7){
            particleShooter.releaseBall();
        } else if (gamepad1.right_trigger > 0.6){
            particleShooter.shoot_loop(true, // fire
                    leftHandPowerDefaultAttenuate + gamepad1.left_trigger*0.5); // boost power
        } else {
            particleShooter.shoot_loop(false, 0.0); // stop
        }

        if (gamepad1.x) {
            particleShooter.reset();
        }
    }

    public void elevatorControl () {
        if (gamepad1.dpad_up && leftArmState == FIRE) {
            robot.motorRightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRightArm.setPower(0.5);
        } else if (gamepad1.dpad_down && leftArmState == FIRE) {
            robot.motorRightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRightArm.setPower(-0.5);
        } else {
            float throttle = gamepad2.right_stick_y;
            if (throttle < 0) {
                throttle /=2; // downward need much less power
            }
            if (Math.abs(throttle) > 0.1) {
                robot.motorRightArm.setPower(Range.clip(throttle, -1.0, 1.0));
            } else {
                robot.motorRightArm.setPower(0.0);
            }
        }
    }

    public void rightHandControl () {

        // extend or retract hand
        float throttle = gamepad2.left_stick_y;

        // tighten the hand by servo

    }

    public void beaconArmControl () {

        if (gamepad1.dpad_left || gamepad2.dpad_left ) {
            leftBeaconArm.state = 2;
        } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
            rightBeaconArm.state = 2;
        } else {
            leftBeaconArm.state = 0;
            rightBeaconArm.state =0;
        }
        leftBeaconArm.pressButton_loop (false);
        rightBeaconArm.pressButton_loop(false);
    }

    void scooperControl () {
        if (gamepad1.left_bumper && leftArmState == LOAD) {
            robot.servoLeftScooper.setPower(leftScooperGo);
            robot.servoRightScooper.setPower(rightScooperGo);
        } else if (gamepad2.left_bumper) {
            robot.servoLeftScooper.setPower(leftScooperGo);
        } else if (gamepad2.right_bumper) {
            robot.servoRightScooper.setPower(rightScooperGo);
        } else {
            robot.servoLeftScooper.setPower(leftScooperStop);
            robot.servoRightScooper.setPower(rightScooperStop);
        }
    }

    void enableLeftArm (){
        boolLeftArmEnable = true;
        boolRightArmEnable = false;
    }

    void enableRightArm() {
        boolLeftArmEnable = false;
        boolRightArmEnable = true;
    }

    private void initBeaconArms () {
        leftBeaconArm = new HardwareBeaconArm("leftBeaconUpperArm", "leftBeaconLowerArm",
                "leftBeaconColor", leftBeaconColorSensorAddr, "leftBeaconTouch");
        leftBeaconArm.init(hardwareMap);
        leftBeaconArm.start(leftUpHomePosition,leftLowHomePosition,leftUpStepSize,leftLowStepSize);
        leftBeaconArm.retract();

        rightBeaconArm = new HardwareBeaconArm("rightBeaconUpperArm", "rightBeaconLowerArm",
                "rightBeaconColor",rightBeaconColorSensorAddr, "rightBeaconTouch");
        rightBeaconArm.init(hardwareMap);
        rightBeaconArm.start(rightUpHomePosition,rightLowHomePosition,rightUpStepSize,rightLowStepSize);
        rightBeaconArm.retract();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.stop();
    }

    public void stopLeftArm () {
        robot.motorLeftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLeftArm.setPower(0.0);
    }

}
