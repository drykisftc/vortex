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

@TeleOp(name="TeleOp: base", group="TeleOp")
public class VortexTeleOp extends OpMode{

    /* Declare OpMode members. */
    HardwareVortex robot       = new HardwareVortex(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwareVortexMatrix class.

    boolean boolLeftArmEnable = true;
    boolean boolRightArmEnable = false;

    int leftArmHomePosition = 0;
    int rightArmHomePosition = 0;
    int armHomeBufferOffset = 20;

    final int leftArmLoadPositionOffset = 500;
    final int leftArmSnapPositionOffset = 50;
    final int leftArmShootPositionOffset = 2000;

    int leftArmLoadPosition = leftArmLoadPositionOffset;
    int leftArmSnapPosition= leftArmSnapPositionOffset;
    int leftArmShootPosition = leftArmShootPositionOffset;

    int leftHandHomePosition = 0;
    int leftHandFirePositionOffset = 400;
    double leftHandHoldPower = 0.1;
    double leftHandFirePower = 1.0;

    enum LeftArmState {
        HOME,
        LOAD,
        FIRE
    }

    private LeftArmState leftArmState = LeftArmState.HOME;

    boolean ArmTargetPositionSet = false;
    int armTargetPosition = 0;

    double [] wheelPowerLUT = {0.0f, 0.05f, 0.15f, 0.18f, 0.20f,
            0.22f, 0.24f, 0.26f, 0.28f, 0.30f, 0.32f, 0.34f, 0.36f,
            0.38f, 0.42f, 0.46f, 0.50f, 0.54f, 0.58f, 0.62f, 0.66f,
            0.70f, 0.74f, 0.78f, 0.82f, 0.86f, 0.90f, 0.94f, 0.98f, 1.00f };

    double [] armPowerLUT = { 0.0f, 0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f, 0.07f,
            0.08f, 0.09f, 0.10f, 0.11f, 0.12f, 0.13f, 0.14f, 0.15f,
            0.17f, 0.18f, 0.19f, 0.20f, 0.21f, 0.22f, 0.23f, 0.24f,
            0.25f, 0.26f, 0.27f, 0.28f, 0.29f, 0.30f, 0.31f, 0.32f,
            0.33f, 0.34f, 0.35f, 0.36f, 0.37f, 0.38f, 0.39f, 0.40f,
            0.41f, 0.42f, 0.43f, 0.44f, 0.45f, 0.46f, 0.47f, 0.48f,
            0.49f, 0.50f, 0.51f, 0.52f, 0.53f, 0.54f, 0.55f, 0.56f,
            0.60f, 0.65f, 0.70f, 0.75f, 0.80f, 0.85f, 0.90f, 0.95f, 1.0f };

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

        robot.motorLeftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("TeleOp", "Hello Vortex");    //
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        robot.motorLeftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLeftHand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorLeftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArmHomePosition = robot.motorLeftArm.getCurrentPosition();
        rightArmHomePosition = robot.motorRightArm.getCurrentPosition();
        leftArmState = LeftArmState.HOME;

        leftArmLoadPosition = leftArmHomePosition+leftArmLoadPositionOffset;
        leftArmSnapPosition= leftArmHomePosition+leftArmSnapPositionOffset;
        leftArmShootPosition = leftArmHomePosition+leftArmShootPositionOffset;

        robot.motorLeftHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftHandHomePosition = robot.motorLeftHand.getCurrentPosition();
        robot.motorLeftHand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLeftHand.setTargetPosition(leftHandHomePosition);
        robot.motorLeftHand.setPower(leftHandHoldPower);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        joystickWheelContol();
        joystickArmContorl();
        updateTelemetry(telemetry);
    }

    public void joystickWheelContol () {
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

    public void joystickArmContorl () {

        // enable arm controls
        if (gamepad1.left_bumper) {
            enableLeftArm();
        }
        if (gamepad1.right_bumper) {
            enableRightArm();
        }

        // move arms by power
        float throttle = gamepad1.right_stick_y;
        if (throttle != 0) {
            ArmTargetPositionSet = false;
            double power = Range.clip(VortexUtils.lookUpTableFunc(throttle, armPowerLUT), -1, 1);
            if (boolLeftArmEnable) {
                robot.motorLeftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.motorLeftArm.setPower(power);
                robot.motorRightArm.setPower(0.0);
            }

            if (boolRightArmEnable) {
                robot.motorRightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.motorRightArm.setPower(0.0);
                robot.motorRightArm.setPower(power);
            }
        }
        else {
            // hold current position
            if (!ArmTargetPositionSet) {
                if (boolLeftArmEnable) {
                    armTargetPosition = robot.motorLeftArm.getCurrentPosition();
                    robot.motorLeftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.motorLeftArm.setTargetPosition(armTargetPosition);
                    robot.motorLeftArm.setPower(1.0);

                }
                if (boolRightArmEnable) {
                    armTargetPosition = robot.motorRightArm.getCurrentPosition();
                    robot.motorRightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.motorRightArm.setTargetPosition(armTargetPosition);
                    robot.motorRightArm.setPower(1.0);
                }
                ArmTargetPositionSet = true;
            }

        }

    }

    public void buttonControl () {

        if (gamepad1.x) {
          // go to load position
            leftArmState = LeftArmState.LOAD;
            robot.motorLeftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorLeftArm.setTargetPosition(leftArmLoadPosition);
            robot.motorLeftArm.setPower(1.0);
        } else if (gamepad1.y) {
            // go to shoot position
            leftArmState = LeftArmState.FIRE;
            robot.motorLeftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorLeftArm.setTargetPosition(leftArmShootPosition);
            robot.motorLeftArm.setPower(1.0);

        } if (gamepad1.b) {
            // go to home position
            leftArmState = LeftArmState.HOME;
            robot.motorLeftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorLeftArm.setTargetPosition(leftArmHomePosition);
            robot.motorLeftArm.setPower(1.0);
        }

        switch (leftArmState) {
            case HOME:
                break;
            case LOAD:
                // if trigger snap, move between snap position and load position
                double snapTrigger = gamepad1.left_trigger;
                robot.motorLeftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorLeftArm.setTargetPosition(leftArmSnapPosition
                        + (int)((leftArmLoadPosition - leftArmSnapPosition)*snapTrigger));
                robot.motorLeftArm.setPower(1.0);
                break;
            case FIRE:
                if (gamepad1.right_trigger> 0.5) {
                    // fire
                    robot.motorLeftHand.setTargetPosition(robot.motorLeftHand.getCurrentPosition()+leftHandFirePositionOffset);
                    robot.motorLeftHand.setPower(leftHandFirePower);
                }
                break;
            default:
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
    }

}
