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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name="Auto: base", group="Auto")
@Disabled
public class VortexAutoOp extends GyroTrackerOpMode{

    BeaconPresser beaconPresser = null;
    ParticleShooter particleShooter = null;
    HardwareLineTracker hardwareLineTracker = null;
    HardwareBeaconArm beaconArm = null;
    HardwareWallTracker wallTracker = null;

    double groundBrightness = 0.0;
    double minLineBrightness = 0.02;

    // navigation settings
    int start2FireDistance = 2500; //2500
    int fire2TurnDegree = 70;
    int fire2WallDistance = 7500;
    int wall2TurnDegree = -70;
    int wall2BeaconDistance = 7500;
    int beacon2ParkTurnDegree = -135;
    int beacon2BeaconDistance = 8000;
    int beacon2ParkingDistance =8000;

    // to do: add wall tracker

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        super.init();

        hardwareLineTracker = new HardwareLineTracker();
        hardwareLineTracker.init(hardwareMap, 4);
        groundBrightness = Math.max(minLineBrightness,hardwareLineTracker.getBaseLineBrightness()*2.5);

        initBeaconArm();
        initWallTracker();

    }

    public void initBeaconArm() {
        beaconArm = new HardwareBeaconArm("rightBeaconUpperArm", "rightBeaconLowerArm",
                "rightBeaconColor", "rightBeaconTouch");
        beaconPresser = new BeaconPresser(gyroTracker, beaconArm);
        beaconPresser.setReporter(telemetry);
        beaconPresser.teamColor = 'b';
        beaconArm.upperArmHomePosition = 0.0;
        beaconArm.upperArmStepSize = 0.01;
        beaconArm.lowerArmHomePosition = 1.0;
        beaconArm.lowerArmStepSize = 0.01;
        beaconArm.retract();

    }

    public void initWallTracker() {
        wallTracker.parkingPosition = 0.0;
        wallTracker.park();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        super.init_loop();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        super.start();
        particleShooter.start(0);
        beaconPresser.start(0);
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
                state = gyroTracker.goStraight (0, cruisingTurnGain, cruisingPower,
                        start2FireDistance, state,state+1);
                telemetry.addData("State:", "%02d", state);
                if (state == 1) {
                    // prepare to shoot
                    particleShooter.start(0);
                    robot.motorLeftWheel.setPower(0.0);
                    robot.motorRightWheel.setPower(0.0);
                }
                break;
            case 1:
                // shoot particles
                state = particleShooter.loop(state, state+1);
                break;
            case 2:
                // turn 45 degree
                state = gyroTracker.turn(fire2TurnDegree, inPlaceTurnGain,
                        turningPower,state,state+1);
                telemetry.addData("State:", "%02d", state);
                break;
            case 3:
                // go straight until hit the wall
                state = gyroTracker.goStraight (fire2TurnDegree, cruisingTurnGain,
                        cruisingPower, fire2WallDistance, state,state+1);
                telemetry.addData("State:", "%02d", state);
                break;
            case 4:
                // turn -45 degree back
                state = gyroTracker.turn(fire2TurnDegree+wall2TurnDegree,
                        inPlaceTurnGain,turningPower,state,state+1);
                telemetry.addData("State:", "%02d", state);
                break;
            case 5:
                // go straight until hit first white line
                state = gyroTracker.goStraight (fire2TurnDegree+wall2TurnDegree,
                        cruisingTurnGain, cruisingPower, wall2BeaconDistance, state,state+1);
                telemetry.addData("State:", "%02d", state);

                // check the ods for white line signal
                if (hardwareLineTracker.onWhiteLine(groundBrightness, 2)) {
                    state = 6;
                    gyroTracker.setWheelLandmark();
                    stopWheels();
                    beaconPresser.start(0);
                }
                break;
            case 6:
                // touch beacon
                state = beaconPresser.loop(state, state+1);
                telemetry.addData("State:", "%02d", state);
                if (state == 7) {
                    gyroTracker.setWheelLandmark();
                }
                break;
            case 7:
                // go straight until hit the second white line
                state = gyroTracker.goStraight (fire2TurnDegree+wall2TurnDegree,
                        cruisingTurnGain, cruisingPower, beacon2BeaconDistance, state,state+1);
                telemetry.addData("State:", "%02d", state);

                // check the ods for white line signal
                if (gyroTracker.getWheelLandmarkOdometer() > 1000
                && hardwareLineTracker.onWhiteLine(groundBrightness, 2)) {
                    state = 8;
                    stopWheels();
                    gyroTracker.setWheelLandmark();
                    beaconPresser.start(0);
                }
                break;
            case 8:
                // touch beacon
                state = beaconPresser.loop(state, state+1);
                telemetry.addData("State:", "%02d", state);
                if (state == 9) {
                    gyroTracker.setWheelLandmark();
                }
                break;
            case 9:
                // turn 135 degree
                state = gyroTracker.turn(fire2TurnDegree+wall2TurnDegree+beacon2ParkTurnDegree,
                        inPlaceTurnGain,turningPower,state,state+1);
                telemetry.addData("State:", "%02d", state);
                break;
            case 10:
                // go straight to central parking
                state = gyroTracker.goStraight (fire2TurnDegree+wall2TurnDegree+beacon2ParkTurnDegree,
                        cruisingTurnGain, cruisingPower, beacon2ParkingDistance, state,state+1);
                telemetry.addData("State:", "%02d", state);
                break;
            case 11:
                // use color strips to help parking
                state = 12;
                break;
            default:
                // stop
                telemetry.addData("State:", "End");
                stop();
        }
        telemetry.update();
    }

    public void stopWheels() {
        robot.motorLeftWheel.setPower(0.0);
        robot.motorRightWheel.setPower(0.0);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        homeArm();
        super.stop();
    }

}
