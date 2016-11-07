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
public class VortexTeleOp extends OpMode{

    /* Declare OpMode members. */
    protected HardwareVortex robot       = new HardwareVortex(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwareVortexMatrix class.

    HardwareWallTracker wallTracker = new HardwareWallTracker();

    protected boolean boolLeftArmEnable = true;
    protected boolean boolRightArmEnable = false;

    protected int leftArmHomePosition = 0;
    protected int rightArmHomePosition = 0;

    protected final int leftArmHomeParkingOffset = 150;
    protected final int leftArmLoadPositionOffset = 950;
    protected final int leftArmSnapPositionOffset = 50;
    protected final int leftArmFirePositionOffset = 4500;
    protected final int leftArmMaxOffset = 4550;
    protected int leftArmFiringSafeZoneOffset = 2000;

    protected int leftArmHomeParkingPostion = leftArmHomeParkingOffset;
    protected int leftArmLoadPosition = leftArmLoadPositionOffset;
    protected int leftArmSnapPosition= leftArmSnapPositionOffset;
    protected int leftArmFirePosition = leftArmFirePositionOffset;
    protected int leftArmFiringSafeZone = leftArmFiringSafeZoneOffset;
    protected int leftArmPeakPosition = leftArmMaxOffset/2;
    protected int leftArmMaxRange = leftArmMaxOffset;

    protected double leftArmJoystickDeadZone = 0.05;
    protected double leftArmHoldPower = 0.3;
    protected double leftArmAutoMovePower = 0.5;
    protected double leftArmAutoSlowMovePower = 0.2;
    protected double leftArmHomingMovePower = -0.3;
    protected long leftArmHomingTimestamp =0;
    protected long leftArmHomingTime =8000;

    // hand parameters
    protected int leftHandHomePosition = 0;
    protected int leftHandFirePositionOffset = 450; // 20: 1 motor is 560. 16:1 is 450
    protected int leftHandFireOvershotOffset = 230; // 20:1 motor is 350. 16:1 is 230
    protected double leftHandHoldPower = 0.1;
    protected double leftHandFirePower = 1.0;
    protected int fireCount =0;
    protected long lastFireTimeStamp = 0;
    protected long minFireInterval = 1800;
    protected long minReloadInterval = 1200;
    protected boolean leftHandReloaded = true;
    protected int leftHandFirePositionTolerance = 10;

    enum LeftArmState {
        HOME,
        LOAD,
        FIRE,
        MANUAL
    }

    protected LeftArmState leftArmState = LeftArmState.HOME;

    boolean isArmHoldingPositionSet = false;
    int armTargetPosition = 0;

    double [] wheelPowerLUT = {0.0f, 0.05f, 0.15f, 0.18f, 0.20f,
            0.22f, 0.24f, 0.26f, 0.28f, 0.30f, 0.32f, 0.34f, 0.36f,
            0.38f, 0.42f, 0.46f, 0.50f, 0.54f, 0.58f, 0.62f, 0.66f,
            0.70f, 0.74f, 0.78f, 0.82f, 0.86f, 0.90f, 0.94f, 0.98f,
            1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f,
            1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f};

    double [] armPowerLUT = { 0.0f, 0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f, 0.07f,
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
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        wallTracker.init(hardwareMap);
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

        // set time stamp
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
        // safety features
        // move wall tracker arm, don't crash into left arm!
        // safe zone already, park
        wallTracker.park();

        // homing the left arm. If the touch sensor is on, turn off arm power
        if (!robot.armStop.isPressed()
                && System.currentTimeMillis() - leftArmHomingTimestamp < leftArmHomingTime) {
            robot.motorLeftArm.setPower(leftArmHomingMovePower);
        } else {
            robot.motorLeftArm.setPower(0.0);
        }
    }

    public void init_loop2() {
        // safety features
        // move wall tracker arm, don't crash into left arm!
        if (wallTracker.sonicArm.getPosition() > 0.56) {
            // safe zone already, park
            wallTracker.park();

            // if touch sensor is on, turn off arm power
            if (robot.armStop.isPressed()) {
                robot.motorLeftArm.setPower(0.0);
            } else if (System.currentTimeMillis() - leftArmHomingTimestamp < leftArmHomingTime) {
                // move arm back to home position
                robot.motorLeftArm.setPower(leftArmHomingMovePower * 0.5);
            } else {
                robot.motorLeftArm.setPower(0.0);
            }
        } else if (System.currentTimeMillis() - leftArmHomingTimestamp < leftArmHomingTime *0.3) {
            // danger zone, lift arm
            robot.motorLeftArm.setPower(-1.0*leftArmHomingMovePower);
        } else if (System.currentTimeMillis() - leftArmHomingTimestamp < leftArmHomingTime *0.6) {
            // park sonic arm
            wallTracker.park();
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
        leftArmHomePosition = robot.motorLeftArm.getCurrentPosition();
        telemetry.addData("left arm home",  "%2d", leftArmHomePosition);
        rightArmHomePosition = robot.motorRightArm.getCurrentPosition();
        leftArmState = LeftArmState.HOME;

        leftArmHomeParkingPostion = leftArmHomePosition + leftArmHomeParkingOffset;
        leftArmLoadPosition = leftArmHomePosition+leftArmLoadPositionOffset;
        leftArmSnapPosition= leftArmHomePosition+leftArmSnapPositionOffset;
        leftArmFirePosition = leftArmHomePosition+ leftArmFirePositionOffset;
        leftArmFiringSafeZone = leftArmHomePosition+ leftArmFiringSafeZoneOffset;
        leftArmPeakPosition = leftArmHomePosition + leftArmMaxOffset/2;
        leftArmMaxRange = leftArmHomePosition + leftArmMaxOffset;

        // hands
        robot.motorLeftHand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLeftHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftHandHomePosition = robot.motorLeftHand.getCurrentPosition();
        robot.motorLeftHand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLeftHand.setTargetPosition(leftHandHomePosition);
        robot.motorLeftHand.setPower(leftHandHoldPower);
        fireCount = 0;

        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        joystickWheelControl();
        joystickArmControl();
        buttonControl();
        triggerControl();
        telemetry.update();
    }

    public void joystickWheelControl() {
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

    public void joystickArmControl() {

        // enable arm controls
        if (gamepad1.left_bumper) {
            enableLeftArm();
        }
        if (gamepad1.right_bumper) {
            enableRightArm();
        }

        // move arms by power
        float throttle = gamepad1.right_stick_y;

        telemetry.addData("left arm",  "%.2f", throttle);
        if (Math.abs(throttle) >= leftArmJoystickDeadZone) {
            // set to manual mode
            leftArmState = LeftArmState.MANUAL;
            isArmHoldingPositionSet = false;

            // get joystick position
            double power = Range.clip(VortexUtils.lookUpTableFunc(throttle, armPowerLUT), -1, 1);
            telemetry.addData("arm power", "%.2f", power);

            // move arms
            robot.motorLeftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            int currentPos = robot.motorLeftArm.getCurrentPosition();
            telemetry.addData("left arm pos", "%6d vs %6d", currentPos, leftArmMaxRange);
            // button x allow arm moving to home, just in case the arm loses encoder home position
            // giving a chance to reset home position
            if ((!gamepad1.x)
                    && robot.armStop.isPressed()
                    && power < 0.0) {
                telemetry.addData("left arm min exceed", "STOP!!!!!!!!!!!!!!!");
                robot.motorLeftArm.setPower(0.0);
                robot.motorRightArm.setPower(0.0);
            } else if (currentPos > leftArmMaxRange
                    && power > 0.0) {
                telemetry.addData("left arm max exceeded", "STOP!!!!!!!!!!!!!!");
                robot.motorLeftArm.setPower(0.0);
                robot.motorRightArm.setPower(0.0);
            } else {
                robot.motorLeftArm.setPower(power);
                robot.motorRightArm.setPower(0.0);
            }


            if (boolRightArmEnable) {
                // do something here
            }
        }
        else {
            switch (leftArmState) {
                case HOME: {
                    // go to home position
                    int currentP = robot.motorLeftArm.getCurrentPosition();
                    if (currentP <= leftArmHomeParkingPostion) {
                        robot.motorLeftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorLeftArm.setPower(0.0);
                    } else if (currentP < leftArmPeakPosition) {
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
                        VortexUtils.moveMotorByEncoder(robot.motorLeftArm, target, 1.0);
                    } else {
                        VortexUtils.moveMotorByEncoder(robot.motorLeftArm, leftArmLoadPosition, leftArmAutoMovePower);
                    }
                }
                    break;
                case FIRE:
                    // move to fire position
                {
                    VortexUtils.moveMotorByEncoder(robot.motorLeftArm, leftArmFirePosition, leftArmAutoMovePower);
                }
                    break;
                case MANUAL:
                default:
                    // hold current position
                    if (isArmHoldingPositionSet) {
                        VortexUtils.moveMotorByEncoder(robot.motorLeftArm, armTargetPosition, leftArmHoldPower);
                    } else {
                        // set the target position to limits.
                        int curPos = robot.motorLeftArm.getCurrentPosition();
                        if ( curPos >= leftArmHomeParkingPostion) {
                            armTargetPosition = Math.max(leftArmHomeParkingPostion, curPos);
                            armTargetPosition = Math.min(leftArmMaxRange, armTargetPosition);
                            isArmHoldingPositionSet = true;
                        }
                        // stop motor to prevent it overshoot the limits. Power will enable in next loop call.
                        robot.motorLeftArm.setPower(0.0);
                    }
            }
        }

        if (gamepad1.dpad_left) {
            wallTracker.moveSonicArmToLeft();
        }

        if (gamepad1.dpad_right) {
            int currentP = robot.motorLeftArm.getCurrentPosition();
            if (currentP > leftArmLoadPosition) {
                wallTracker.moveSonicArmToRight();
            }
        }
    }

    public void buttonControl () {
        if (gamepad1.a) {
            // go to home position
            leftArmState = LeftArmState.HOME;
        } else if (gamepad1.b) {
            // go to load position
            leftArmState = LeftArmState.LOAD;
        } else if (gamepad1.y) {
            // go to shoot position
            leftArmState = LeftArmState.FIRE;
        } else if ( gamepad1.x && gamepad1.y )
        {
            // reset left arm home position when both x and y button is pressed
            robot.motorLeftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void triggerControl () {
        long currentT = System.currentTimeMillis();
        long timeSinceLastFiring = currentT - lastFireTimeStamp;
        // firing trigger
        if (gamepad1.right_trigger > 0.3) {
            int currentP = robot.motorLeftArm.getCurrentPosition();
            int fireP = leftHandHomePosition + fireCount * leftHandFirePositionOffset;
            if (leftHandReloaded
                    && timeSinceLastFiring > minFireInterval) {
                if (currentP > leftArmFiringSafeZone) {
                    // fire
                    fireCount++;
                    lastFireTimeStamp = currentT;
                    VortexUtils.moveMotorByEncoder(robot.motorLeftHand,
                            fireP + leftHandFireOvershotOffset + leftHandFirePositionOffset,
                            leftHandFirePower);
                    // encoder mode is slow. just use full power.
                    //robot.motorLeftHand.setPower(1.0);
                }
            } else if (timeSinceLastFiring > minReloadInterval) {
                // reload
                VortexUtils.moveMotorByEncoder(robot.motorLeftHand,
                        fireP, leftHandFirePower * 0.5);
            } else if (Math.abs(currentP - fireP) < leftHandFirePositionTolerance) {
                // assume that reload is done
                leftHandReloaded = true;
            }
        } else {
            VortexUtils.moveMotorByEncoder(robot.motorLeftHand,
                    leftHandHomePosition + fireCount * leftHandFirePositionOffset,
                    leftHandHoldPower);
            leftHandReloaded = true;
        }
    }

    public void enableLeftArm (){
        boolLeftArmEnable = true;
        boolRightArmEnable = false;
    }

    public void enableRightArm() {
        boolLeftArmEnable = false;
        boolRightArmEnable = true;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.stop();
    }

}
