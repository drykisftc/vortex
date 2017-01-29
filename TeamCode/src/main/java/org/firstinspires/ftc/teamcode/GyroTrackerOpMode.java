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

@Autonomous(name="Auto: gyro tracker", group="Testing")
public class GyroTrackerOpMode extends VortexTeleOp {

    /* Declare OpMode members. */
    HardwareGyroTracker gyroTrackerHW = null;
    GyroTracker gyroTracker = null;

    // state machine
    int state = 0;

    int bufferSize= 10;

    // navigation path info
    int testDistance1 = 6000; //2500
    int testDistance2 = 6000;
    int testTurnAngle1 = 90;
    int testTurnAngle2 = 180;
    int backDistance = -6000;

    // navigation control info
    double chargingPower = 0.9;
    double cruisingPower = 0.45;
    double searchingPower = 0.15;
    double cruisingTurnGain = 0.01;
    double inPlaceTurnGain = 0.005;
    double turningPower = 0.0; // set to 0.0 to turn in-place
    double parkTurningPower = -0.1;

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
        gyroTracker.minTurnPower = 0.01;
        gyroTracker.maxTurnPower = 0.35;
        gyroTracker.skewPowerGain = 1.0; // 180 for track wheels
        gyroTracker.skewTolerance = 0;

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
        super.start();
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
        telemetry.addData("skew gain: ", gyroTracker.skewPowerGain);
        telemetry.addData("skew tolerance: ", gyroTracker.skewTolerance);
        telemetry.addData("min turn power: ", gyroTracker.minTurnPower);
        telemetry.addData("max turn power: ", gyroTracker.maxTurnPower);

        switch (state) {
            case 0:
                // go straight
                gyroTracker.skewTolerance = 0;
                state = gyroTracker.goStraight (0, cruisingTurnGain,
                        cruisingPower, testDistance1, state, state+1);
                break;
            case 1:
                // turn 90 degree
                gyroTracker.skewTolerance = 2;
                state = gyroTracker.turn(testTurnAngle1, inPlaceTurnGain,turningPower,state, state+1);
                break;
            case 2:
                // go straight
                gyroTracker.skewTolerance = 0;
                state = gyroTracker.goStraight (testTurnAngle1, cruisingTurnGain,
                        searchingPower, testDistance1,state, state+1);
                break;
            case 3:
                // turn 90 degree
                gyroTracker.skewTolerance = 2;
                state = gyroTracker.turn(testTurnAngle1*2, inPlaceTurnGain,turningPower,state, state+1);
                break;
            case 4:
                // go straight
                gyroTracker.skewTolerance = 0;
                state = gyroTracker.goStraight (testTurnAngle1*2, cruisingTurnGain,
                        chargingPower, testDistance1, state, state+1);
                break;
            case 5:
                // turn 45 degree
                gyroTracker.skewTolerance = 2;
                state = gyroTracker.turn(testTurnAngle1*3, inPlaceTurnGain,turningPower,state, state+1);
                break;
            case 6:
                // go straight
                gyroTracker.skewTolerance = 0;
                state = gyroTracker.goStraight (testTurnAngle1*3, cruisingTurnGain,
                        cruisingPower, testDistance1, state, state+1);
                break;
            case 7:
                // backup
                gyroTracker.skewTolerance = 0;
                state = gyroTracker.goStraight (testTurnAngle1*3, cruisingTurnGain,
                        -1.0*cruisingPower, backDistance, state, state+1);
                break;
            case 8:
                // turn 180 degree
                gyroTracker.skewTolerance = 2;
                state = gyroTracker.turn(testTurnAngle1*3+testTurnAngle2, inPlaceTurnGain,turningPower,state, state+1);
                break;
            case 9:
                // backup
                gyroTracker.skewTolerance = 0;
                state = gyroTracker.goStraight (testTurnAngle1*3+testTurnAngle2, cruisingTurnGain,
                        -1.0*cruisingPower, backDistance, state, state+1);
                break;
            case 10:
                // turn 45 degree
                gyroTracker.skewTolerance = 2;
                state = gyroTracker.turn(testTurnAngle1*2+testTurnAngle2, inPlaceTurnGain,turningPower,state, state+1);
                break;
            case 11:
                gyroTracker.skewTolerance = 0;
                state = gyroTracker.goStraight (testTurnAngle1*2+testTurnAngle2, cruisingTurnGain,
                        cruisingPower, testDistance1, state, state+1);
                break;
            case 12:
                // turn 45 degree
                gyroTracker.skewTolerance = 2;
                state = gyroTracker.turn(testTurnAngle1*3+testTurnAngle2, inPlaceTurnGain,turningPower,state, state+1);
                break;
            default:
                homeArm();
                stop();
        }
        telemetry.update();
    }

    public void raiseArm () {
        VortexUtils.moveMotorByEncoder(robot.motorLeftArm, leftArmMovePosition, leftArmAutoMovePower);

    }

    public void homeArm () {
        VortexUtils.moveMotorByEncoder(robot.motorLeftArm, leftArmHomeParkingPostion, 0.1);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    public void stop() {

        gyroTracker.stop();
    }

}
