/*
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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

import java.nio.Buffer;


/**
 *
 */

@Autonomous(name="Vortex: line tracker", group="LineTracker")
public class LineTrackerBot_Linear extends OpMode {

    /* Declare OpMode members. */
    HardwareLineTracker         robot   = new HardwareLineTracker();   // Use a Pushbot's hardware

    // state machine
    int state = 0;

    int bufferSize= 10;
    double [] leftBuffer = null;
    double [] rightBuffer = null;
    double [] middleBuffer = null;
    int bufferIndex =0;
    Double baselineODS =0.0d;

    double[] ods2PowerLUT = {0.0f, 0.05f, 0.15f, 0.18f, 0.20f,
            0.22f, 0.24f, 0.26f, 0.28f, 0.30f, 0.32f, 0.34f, 0.36f,
            0.38f, 0.42f, 0.46f, 0.50f, 0.54f, 0.58f, 0.62f, 0.66f,
            0.70f, 0.74f, 0.78f, 0.82f, 0.86f, 0.90f, 0.94f, 0.98f, 1.00f };

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        leftBuffer = new double[bufferSize];
        rightBuffer = new double[bufferSize];
        middleBuffer = new double[bufferSize];

        robot.leftODS.enableLed(true);
        robot.rightODS.enableLed(true);
        robot.middleODS.enableLed(true);

        calibrate();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello LineTracker");    //
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // collect baseline brightness
        calibrate();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        // compute baseline brightness
        baselineODS = getBaselineODS();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (state) {
            case 0:
                // follow line
                followLine(0.2);
                break;
            case 1:
                // search for line
                break;
            default:
                break;
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.motorLeftWheel.setPower(0.0);
        robot.motorRightWheel.setPower(0.0);
    }

    public void readODS () {

        leftBuffer[bufferIndex] = robot.leftODS.getLightDetected();
        rightBuffer[bufferIndex] = robot.rightODS.getLightDetected();
        middleBuffer[bufferIndex] = robot.middleODS.getLightDetected();
        bufferIndex++;
        if (bufferIndex>=bufferSize) {
            bufferIndex = 0;
        }
    }

    public double getBaselineODS () {
        double bl =0;
        for (int i =0; i < bufferSize; i++) {
            bl += leftBuffer[i];
            bl += rightBuffer[i];
            bl += middleBuffer[i];
        }

        return bl/3.0/bufferSize;
    }

    public void calibrate () {

        for (int i =0;i < bufferSize; i++){
            readODS();
        }

    }

    public void followLine(double power) {

        double l = robot.leftODS.getLightDetected();
        double r = robot.rightODS.getLightDetected();
        double delta = Range.clip(l-r, -1, 1);
        double direction = ResQUtils.lookUpTableFunc(delta, ods2PowerLUT);
        double left = Range.clip(power+direction, -1,1);
        double right = Range.clip(power-direction, -1,1);
        robot.motorLeftWheel.setPower(left);
        robot.motorRightWheel.setPower(right);
    }

}
