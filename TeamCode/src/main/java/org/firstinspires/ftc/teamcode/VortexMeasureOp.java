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

@TeleOp(name="TeleOp: measure", group="TeleOp")
public class VortexMeasureOp extends VortexTeleOp {

    HardwareLineTracker lineTracker = new HardwareLineTracker();
    HardwareWallTracker wallTracker = new HardwareWallTracker();
    HardwareGyroTracker gyroTracker = new HardwareGyroTracker();

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
        gyroTracker.init(robot.hwMap);
        lineTracker.init(robot.hwMap);
        wallTracker.init(robot.hwMap);

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
        if (gyroTracker.gyro.isCalibrating())  {
            telemetry.addData(">", "Gyro is calibrating.  DO NOT start.");
        }
        else {
            telemetry.addData(">", "Gyro calibrated.  Press Start.");
        }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        gyroTracker.gyro.resetZAxisIntegrator();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        super.loop();

        // get wheel info
        telemetry.addData("Left wheel pos", robot.motorLeftWheel.getCurrentPosition());
        telemetry.addData("right wheel pos", robot.motorRightWheel.getCurrentPosition());

        // get arm info
        telemetry.addData("left arm pos", robot.motorLeftArm.getCurrentPosition());
        telemetry.addData("right arm pos", robot.motorRightArm.getCurrentPosition());

        // get hand info
        telemetry.addData("left hand pos",robot.motorLeftHand.getCurrentPosition());
        telemetry.addData("right hand pos", robot.motorRightHand.getCurrentPosition());

        // get ods info
        for (int i =0; i< lineTracker.arraySize; i++) {
            telemetry.addData("ODS Raw" + Integer.toString(i), lineTracker.sensorArray[i].getRawLightDetected());
            telemetry.addData("ODS Normal"+ Integer.toString(i), lineTracker.sensorArray[i].getLightDetected());
        }

        // get range info
        telemetry.addData("Range left raw ultrasonic", wallTracker.leftRange.rawUltrasonic());
        telemetry.addData("Range left raw optical", wallTracker.leftRange.rawOptical());
        telemetry.addData("Range left cm optical", "%.2f cm", wallTracker.leftRange.cmOptical());
        telemetry.addData("Range left cm", "%.2f cm", wallTracker.leftRange.getDistance(DistanceUnit.CM));
        telemetry.addData("Range right raw ultrasonic", wallTracker.leftRange.rawUltrasonic());
        telemetry.addData("Range rightraw optical", wallTracker.leftRange.rawOptical());
        telemetry.addData("Range rightcm optical", "%.2f cm", wallTracker.leftRange.cmOptical());
        telemetry.addData("Range rightcm", "%.2f cm", wallTracker.leftRange.getDistance(DistanceUnit.CM));

        // get gyro info
        curResetState = (gamepad1.a && gamepad1.b);
        if(curResetState && !lastResetState)  {
            gyroTracker.gyro.resetZAxisIntegrator();
        }
        lastResetState = curResetState;

        xVal = gyroTracker.gyro.rawX();
        yVal = gyroTracker.gyro.rawY();
        zVal = gyroTracker.gyro.rawZ();

        heading = gyroTracker.gyro.getHeading();
        angleZ  = gyroTracker.gyro.getIntegratedZValue();

        telemetry.addData(">", "Press A & B to reset Heading.");
        telemetry.addData("0", "Heading %03d", heading);
        telemetry.addData("1", "Int. Ang. %03d", angleZ);
        telemetry.addData("2", "X av. %03d", xVal);
        telemetry.addData("3", "Y av. %03d", yVal);
        telemetry.addData("4", "Z av. %03d", zVal);

        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        super.stop();
    }

}