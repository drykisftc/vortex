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

@Autonomous(name="Plan G: Red", group="Plan G")
public class PlanGRedAutoOp extends VortexAutoOp{

    int leftArmHitBallPosition = 400;
    protected int fireToBallDistance = 4000;
    protected int ballToParkDistance = 3500;


    @Override
    public void start() {
        super.start();
        start2FireDistance = 3800; //2500 per block
        fire2TurnDegree = 45;
        fire2WallDistance = 6800;
        wall2TurnDegree = -45;
        wall2BeaconDistance = 7500;
        beacon2ParkTurnDegree = -45;
        beacon2BeaconDistance = 4000;
        beacon2ParkingDistance =8000;

        turningPower = 0.02;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (state) {
            case 0:
                if (System.currentTimeMillis() - lastTimeStamp > 10000) {
                    state = 1;
                }
                break;
            case 1:
                // go straight
                particleShooter.moveArmToFirePosition();
                state = gyroTracker.goStraight(0, cruisingTurnGain, cruisingPower,
                        start2FireDistance, state, state + 1);
                telemetry.addData("State:", "%02d", state);
                if (state == 2) {
                    // prepare to shoot
                    robot.motorLeftWheel.setPower(0.0);
                    robot.motorRightWheel.setPower(0.0);
                    particleShooter.start(0);
                }
                break;
            case 2:
                // shoot particles
                state = particleShooter.loop(state, state + 1);
                break;
            case 3:
                VortexUtils.moveMotorByEncoder(robot.motorLeftArm,
                        leftArmHitBallPosition, leftArmAutoMovePower);
                state = gyroTracker.turn(fire2TurnDegree,
                        inPlaceTurnGain, turningPower, state, state + 1);
                telemetry.addData("State:", "%02d", state);
                break;
            case 4:
                state = gyroTracker.turn(fire2TurnDegree,
                        inPlaceTurnGain, turningPower, state, state + 1);
                telemetry.addData("State:", "%02d", state);
                break;
            case 5:
                state = gyroTracker.goStraight(fire2TurnDegree, cruisingTurnGain,
                        cruisingPower, ballToParkDistance, state, state + 1);
                telemetry.addData("State:", "%02d", state);
                break;
            case 6:
                state = gyroTracker.turn(beacon2ParkTurnDegree,
                        inPlaceTurnGain, turningPower, state, state + 1);
                telemetry.addData("State:", "%02d", state);
                break;
            case 7:
                state = gyroTracker.goStraight (ballToParkDistance,
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
                state = beaconPresser.loop(state, state+1);
                telemetry.addData("State:", "%02d", state);
                if (state == 9) {
                    gyroTracker.setWheelLandmark();
                }
                break;
            case 9:
                state = gyroTracker.goStraight (beacon2ParkTurnDegree+180,
                        cruisingTurnGain, 0.175, beacon2ParkingDistance, state,state+1);
                telemetry.addData("State:", "%02d", state);
                break;
            default:
                // stop
                telemetry.addData("State:", "End");
                stop();
        }
        telemetry.update();
    }
}
