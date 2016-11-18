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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

@TeleOp(name="TeleOp: Z measure", group="Testing")
public class VortexMeasureOp extends VortexTeleOp {

    HardwareLineTracker lineTracker = new HardwareLineTracker();
    HardwareGyroTracker gyroTrackerHW = new HardwareGyroTracker();

    int xVal, yVal, zVal = 0;     // Gyro rate Values
    int heading = 0;              // Gyro integrated heading
    int angleZ = 0;
    boolean lastResetState = false;
    boolean curResetState  = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        super.init();
        gyroTrackerHW.init(robot.hwMap);
        lineTracker.init(robot.hwMap,4);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("MeasureOp", "Hello Vortex");    //
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // make sure the gyro is calibrated.
        if (gyroTrackerHW.gyro.isCalibrating())  {
            telemetry.addData(">", "Gyro is calibrating.  DO NOT start!!!!");
            telemetry.addData(">", "NO! NO! NO! Don't! Don't! Don't! Wait! Wait! Wait! ");
            telemetry.addData(">", "NO! NO! NO! Don't! Don't! Don't! Wait! Wait! Wait! ");
            telemetry.addData(">", "NO! NO! NO! Don't! Don't! Don't! Wait! Wait! Wait! ");
            telemetry.addData(">", "NO! NO! NO! Don't! Don't! Don't! Wait! Wait! Wait! ");
            telemetry.addData(">", "NO! NO! NO! Don't! Don't! Don't! Wait! Wait! Wait! ");
        }
        else {
            telemetry.addData(">", "Press Start.");
        }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        gyroTrackerHW.gyro.resetZAxisIntegrator();

        robot.motorLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorLeftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLeftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorLeftHand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLeftHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // get wheel info
        telemetry.addData("Left wheel pos ", "%6d", robot.motorLeftWheel.getCurrentPosition());
        telemetry.addData("right wheel pos", "%6d", robot.motorRightWheel.getCurrentPosition());

        // get arm info
        telemetry.addData("left arm pos  ", "%6d", robot.motorLeftArm.getCurrentPosition());
        telemetry.addData("right arm pos ", "%6d", robot.motorRightArm.getCurrentPosition());

        // get hand info
        telemetry.addData("left hand pos ", "%6d", robot.motorLeftHand.getCurrentPosition());
        telemetry.addData("right hand pos", "%6d", robot.motorRightHand.getCurrentPosition());

        // get ods info
        for (int i =0; i< lineTracker.arraySize; i++) {
            telemetry.addData("ODS Raw   " + Integer.toString(i), lineTracker.sensorArray[i].getRawLightDetected());
            telemetry.addData("ODS Normal" + Integer.toString(i), lineTracker.sensorArray[i].getLightDetected());
        }

        telemetry.addData("Arm limit switch on  ", "%b", robot.armStop.isPressed());

        // get range info
        telemetry.addData("Range arm position   ", "%.2f", wallTrackerHW.sonicArm.getPosition());
        telemetry.addData("Range raw optical    ", "%3d", wallTrackerHW.sonicRange.rawOptical());
        telemetry.addData("Range cm optical     ", "%.2f cm", wallTrackerHW.sonicRange.cmOptical());
        telemetry.addData("Range raw ultrasonic ", "%3d",  wallTrackerHW.sonicRange.rawUltrasonic());
        telemetry.addData("Range cm ultrasonic  ", "%.2f cm", wallTrackerHW.sonicRange.getDistance(DistanceUnit.CM));

        // get gyro info
        curResetState = (gamepad1.a && gamepad1.b);
        if(curResetState && !lastResetState)  {
            gyroTrackerHW.gyro.resetZAxisIntegrator();
        }
        lastResetState = curResetState;

        xVal = gyroTrackerHW.gyro.rawX();
        yVal = gyroTrackerHW.gyro.rawY();
        zVal = gyroTrackerHW.gyro.rawZ();

        heading = gyroTrackerHW.gyro.getHeading();
        angleZ  = gyroTrackerHW.gyro.getIntegratedZValue();

        telemetry.addData("Gyro", "Press A & B to reset Heading.");
        telemetry.addData("0", "Heading %03d", heading);
        telemetry.addData("1", "Int. Ang. %03d", angleZ);
        telemetry.addData("2", "X av. %03d", xVal);
        telemetry.addData("3", "Y av. %03d", yVal);
        telemetry.addData("4", "Z av. %03d", zVal);

        joystickWheelControl();
        joystickArmControlSimple();

        telemetry.update();
    }

    public void joystickArmControlSimple() {
        if (gamepad1.left_bumper) {
            enableLeftArm();
        }
        if (gamepad1.right_bumper) {
            enableRightArm();
        }

        float throttle = gamepad1.right_stick_y;


        if (boolLeftArmEnable) {
            robot.motorLeftArm.setPower(Range.clip(VortexUtils.lookUpTableFunc(throttle, armPowerLUT),-1,1));
        }

        if (boolRightArmEnable) {
            robot.motorRightArm.setPower(Range.clip(VortexUtils.lookUpTableFunc(throttle, armPowerLUT),-1,1));
        }

        robot.motorLeftHand.setPower(Range.clip(gamepad1.right_trigger, -1, 1));
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        super.stop();
    }


}
