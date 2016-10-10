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


/**
 *
 */

@Autonomous(name="Auto: gyro tracker", group="Testing")
public class GyroTrackerOpMode extends OpMode {

    /* Declare OpMode members. */
    HardwareGyroTracker         robot   = new HardwareGyroTracker();   // Use a Pushbot's hardware

    // state machine
    int state = 0;

    int bufferSize= 10;

    int start2FireDistance = 500;
    int fire2TurnDegree = 45;
    int fire2WallDistance = 3000;
    int wall2TurnDegree = -45;
    int wall2BeaconDistance = 5000;

    GyroTracker gyroTracker = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        gyroTracker = new GyroTracker(robot.gyro,
                robot.motorLeftWheel,
                robot.motorRightWheel,
                bufferSize);
        gyroTracker.init();
        gyroTracker.setReporter(telemetry);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello LineTracker");    //
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
        // compute baseline brightness
        gyroTracker.start(0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (state) {
            case 0:
                // go straight
                state = goStraight (0, 1.0/10.0, 1.0, 0, start2FireDistance, 0,1);
                telemetry.addData("State:", "%02d", state);
                break;
            case 1:
                // turn 45 degree
                state = turn(fire2TurnDegree,0.1,0.0,1,2);
                telemetry.addData("State:", "%02d", state);
                break;
            case 2:
                // go straight
                state = goStraight (0, 1.0/10.0, 1.0, 0, fire2WallDistance, 2,3);
                telemetry.addData("State:", "%02d", state);
                break;
            case 3:
                // turn -45 degree
                state = turn(wall2TurnDegree,0.1,0.0,1,2);
                telemetry.addData("State:", "%02d", state);
                break;
            case 4:
                // go straight
                state = goStraight (0, 1.0/10.0, 1.0, 0, wall2BeaconDistance, 4,5);
                telemetry.addData("State:", "%02d", state);
                break;
            default:
                // stop
                telemetry.addData("State:", "End");
                stop();
        }
        telemetry.update();
    }

    public int goStraight ( int heading, double sensitivity, double power,
                            int startDistance, int deltaDistance,
                            int startState, int endState) {
        // get motor distance
        int lD = robot.motorLeftWheel.getCurrentPosition();
        int rD = robot.motorRightWheel.getCurrentPosition();
        int d = Math.min(lD, rD);

        if ( d - startDistance < deltaDistance) {
            gyroTracker.skewAngelPowerGain = sensitivity;
            gyroTracker.goStraight(heading, power);
            return startState;
        }

        return endState;
    }

    public int turn ( int heading, double sensitivity, double power,
                      int startState, int endState) {
        gyroTracker.skewAngelPowerGain = sensitivity;
        if (gyroTracker.goStraight(heading, power) == true) {
            return endState;
        }
        return startState;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    public void stop() {
        gyroTracker.stop();
    }

}