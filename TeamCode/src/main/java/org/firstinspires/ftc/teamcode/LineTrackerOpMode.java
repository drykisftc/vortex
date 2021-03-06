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
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.nio.Buffer;


/**
 *
 */

@Autonomous(name="Auto: line tracker", group="Testing")
public class LineTrackerOpMode extends OpMode {

    /* Declare OpMode members. */
    HardwareVortex vortexHW = null;
    HardwareLineTracker tracker   = null;

    // state machine
    int state = 0;

    int bufferSize= 10;


    LineTracker lineTracker = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        vortexHW = new HardwareVortex();
        tracker   = new HardwareLineTracker();
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        vortexHW.init(hardwareMap);
        tracker.init(hardwareMap,3);

        lineTracker = new LineTracker(tracker.sensorArray[0],
                tracker.sensorArray[1],
                tracker.sensorArray[2],
                vortexHW.motorLeftWheel,
                vortexHW.motorRightWheel,
                bufferSize);
        lineTracker.init();
        lineTracker.setReporter(telemetry);

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
        lineTracker.calibrate();
        telemetry.addData("BaseLineODS", "="+Double.toString(lineTracker.baselineODS));    //
        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        // compute baseline brightness
        lineTracker.start(0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (state) {
            case 0:
                lineTracker.loop(0.4);
                break;
            default:
        }
        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    public void stop() {
        lineTracker.stop();
    }

}
