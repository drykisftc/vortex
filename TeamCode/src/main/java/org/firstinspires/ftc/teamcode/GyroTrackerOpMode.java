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

    protected final int leftArmRaisedPositionOffset = 1000;
    protected final int leftArmHomePositionOffset = 100;

    // state machine
    int state = 0;

    int bufferSize= 10;

    // landmark info
    int landMarkPosition = 0;
    int landMarkAngle = 0;

    // navigation path info
    int testDistance1 = 7500; //2500
    int testDistance2 = 7500;
    int testTurnAngle1 = 75;
    int testTurnAngle2 = -135;

    // navigation control info
    double cruisingPower = 1.0;
    double searchingPower = 0.3;
    double cruisingTurnGain = 0.05;
    double inPlaceTurnGain = 0.01;
    double turningPower = 0.0; // set to 0.0 to turn in-place

    // arm. Warning, arm power > 0.6 will damage the gear boxes
    double armPower = 0.4;

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
        landMarkPosition = 0;
        landMarkAngle = gyroTracker.gyro.getHeading();
        raiseArm();
        state = 0;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (state) {
            case 0:
                // go straight
                state = goStraight (landMarkAngle, cruisingTurnGain, cruisingPower, landMarkPosition, testDistance1, 0,1);
                telemetry.addData("State:", "%02d", state);
                break;
            case 1:
                // turn 45 degree
                state = turn(landMarkAngle+testTurnAngle1, inPlaceTurnGain,turningPower,1,2);
                telemetry.addData("State:", "%02d", state);
                break;
            case 2:
                // go straight
                state = goStraight (landMarkAngle+testTurnAngle2, cruisingTurnGain, cruisingPower, landMarkPosition, testDistance2, 2,3);
                telemetry.addData("State:", "%02d", state);
                break;
            case 3:
                // turn 45 degree
                state = turn(landMarkAngle+testTurnAngle2, inPlaceTurnGain,turningPower,1,2);
                telemetry.addData("State:", "%02d", state);
                break;
            default:
                // stop
                telemetry.addData("State:", "End");
                homeArm();
                stop();
        }
        telemetry.update();
    }

    public int goStraight ( double heading, double gain, double power,
                            int startDistance, int deltaDistance,
                            int startState, int endState) {
        // get motor distance
        int lD = robot.motorLeftWheel.getCurrentPosition();
        int rD = robot.motorRightWheel.getCurrentPosition();
        int d = Math.min(lD, rD);

        if ( d - startDistance < deltaDistance) {
            gyroTracker.skewAngelPowerGain = gain;
            gyroTracker.maintainHeading(heading, power);
            return startState;
        }
        // go to next state
        landMarkPosition = d;
        robot.motorLeftWheel.setPower(0.0);
        robot.motorRightWheel.setPower(0.0);
        return endState;
    }

    public int turn ( double heading, double sensitivity, double power,
                      int startState, int endState) {
        gyroTracker.skewAngelPowerGain = sensitivity;
        if (gyroTracker.maintainHeading(heading, power) != true) {
            return startState;
        }
        /* got to next state */
        robot.motorLeftWheel.setPower(0.0);
        robot.motorRightWheel.setPower(0.0);
        int lD = robot.motorLeftWheel.getCurrentPosition();
        int rD = robot.motorRightWheel.getCurrentPosition();
        landMarkPosition = Math.min(lD, rD);
        return endState;
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
