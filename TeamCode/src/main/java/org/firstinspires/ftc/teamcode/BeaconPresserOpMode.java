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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 *
 */

@TeleOp(name="TeleOp: beacon presser", group="Testing")
public class BeaconPresserOpMode extends OpMode {

    /* Declare OpMode members. */
    HardwareBeaconArm beaconArm  = null;   // Use a Pushbot's hardware

    // arm control information
    boolean loopTrue = false;
    double upHomePosition = 0.93;
    double upStepSize = -0.026;
    double lowHomePosition = 0.86;
    double lowStepSize = -0.04;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello BeaconPresserOpMode");    //
        updateTelemetry(telemetry);

        beaconArm = new HardwareBeaconArm("leftBeaconUpperArm", "leftBeaconLowerArm",
                "leftBeaconColor", "leftBeaconTouch");
        beaconArm.init(hardwareMap);
        beaconArm.start(upHomePosition,lowHomePosition,upStepSize,lowStepSize);
        beaconArm.retract();
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
        // compute baseline brightness
        beaconArm.state = 0;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (gamepad1.a) {
            beaconArm.state = 0;
        }

        if (gamepad1.b) {
            beaconArm.state = 1;
        }

        if (gamepad1.y) {
            beaconArm.state = 2;
        }

        if (gamepad1.x) {
            if (loopTrue) {
                loopTrue = false;
            } else {
                loopTrue = true;
            }
        }

        beaconArm.pressButton_loop (loopTrue);

        telemetry.addData("Upper Arm Pos   ", beaconArm.upperArm.getPosition());
        telemetry.addData("Lower Arm Pos   ", beaconArm.lowerArm.getPosition());
        telemetry.addData("Color sensor rgb", "%d,%d,%d", beaconArm.colorSensor.red(),
                beaconArm.colorSensor.green(), beaconArm.colorSensor.blue());
        telemetry.addData("Near counts     ", beaconArm.nearCounts);
        telemetry.addData("Touch sensor on ", "%b", beaconArm.touchSensor.isPressed());
        telemetry.addData("Touch counts    ", beaconArm.touchCounts);
        telemetry.addData("State:", "%02d", beaconArm.state);
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    public void stop() {

    }

}
