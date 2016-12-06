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
public class GyroTrackerOpMode extends VortexTeleOp {

    /* Declare OpMode members. */
    HardwareGyroTracker gyroTrackerHW = null;
    GyroTracker gyroTracker = null;

    protected final int leftArmRaisedPositionOffset = 1000;
    protected final int leftArmHomePositionOffset = 100;

    // state machine
    int state = 0;

    int bufferSize= 10;

    // navigation path info
    int testDistance1 = 7500; //2500
    int testDistance2 = 7500;
    int testTurnAngle1 = 75;
    int testTurnAngle2 = -135;

    // navigation control info
    double cruisingPower = 0.4;
    double searchingPower = 0.3;
    double cruisingTurnGain = 0.005;
    double inPlaceTurnGain = 0.005;
    double turningPower = 0.0; // set to 0.0 to turn in-place

    // arm. Warning, arm power > 0.6 will damage the gear boxes
    double armPower = 0.4;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // init gyro first to ensure gyro calibration done
        gyroTrackerHW = new HardwareGyroTracker();
        gyroTrackerHW.init(hardwareMap);

        // init other devices
        super.init();

        gyroTracker = new GyroTracker(gyroTrackerHW.gyro,
                robot.motorLeftWheel,
                robot.motorRightWheel,
                bufferSize);
        gyroTracker.setReporter(telemetry);
        gyroTracker.init();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("GyroTracker", "Init");    //
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        super.init_loop();
        // make sure the gyro is calibrated.
        if (gyroTrackerHW.gyro.isCalibrating())  {
            telemetry.addData("Gyro measuring mode", gyroTrackerHW.gyro.getMeasurementMode());
            telemetry.addData(">", "Gyro is calibrating.  DO NOT start!!!!");
            telemetry.addData(">", "Wait! Wait! Wait! ");
            telemetry.addData(">", "Wait! Wait! Wait! Wait!");
            telemetry.addData(">", "Wait! Wait! Wait! Wait! Wait!");
            telemetry.addData(">", "Wait! Wait! Wait! Wait!Wait!");
            telemetry.addData(">", "Wait! Wait! Wait! Wait!");
            telemetry.addData(">", "Wait! Wait! Wait! ");
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
        // compute baseline brightness
        gyroTracker.start(0);
        raiseArm();
        state = 0;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("State:", "%02d", state);
        switch (state) {
            case 0:
                // go straight
                state = gyroTracker.goStraight (0, cruisingTurnGain, cruisingPower, testDistance1, 0,1);
                break;
            case 1:
                // turn 45 degree
                state = gyroTracker.turn(testTurnAngle1, inPlaceTurnGain,turningPower,1,2);
                break;
            case 2:
                // go straight
                state = gyroTracker.goStraight (testTurnAngle2, cruisingTurnGain, cruisingPower, testDistance2, 2,3);
                break;
            case 3:
                // turn 45 degree
                state = gyroTracker.turn(testTurnAngle2, inPlaceTurnGain,turningPower,1,2);
                break;
            default:
                homeArm();
                stop();
        }
        telemetry.update();
    }

    public void raiseArm () {
        VortexUtils.moveMotorByEncoder(robot.motorLeftArm, leftArmRaisedPositionOffset, armPower);

    }

    public void homeArm () {
        VortexUtils.moveMotorByEncoder(robot.motorLeftArm, leftArmHomePositionOffset, 0.1);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    public void stop() {

        gyroTracker.stop();
    }

}
