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

@Autonomous(name="Plan D: Red", group="Plan D")
public class PlanDRedAutoOp extends PlanARedAutoOp{

    @Override
    public void loop() {
        telemetry.addData("State:", "%02d", state);
        telemetry.addData("Wall Distance: ", "%02f", wallTracker.wallTrackerHW.getDistance());
        switch (state) {
            case 0:
                // go straight
                gyroTracker.skewTolerance = 0;
                state = gyroTracker.goStraight (0, cruisingTurnGain, cruisingPower,
                        start2FireDistance, state,state+1);

                if (System.currentTimeMillis() - lastTimeStamp > 200) {
                    // move and raise arm at same time
                    VortexUtils.moveMotorByEncoder(robot.motorLeftArm,
                            leftArmFirePosition, leftArmFastAutoMovePower);
                    state = gyroTracker.goStraight (0, cruisingTurnGain, cruisingPower,
                            start2FireDistance, state,state+1);
                    particleShooter.reload();
                    particleShooter.relaxHand();
                } else {
                    // slow start to avoid turning
                    state = gyroTracker.goStraight (0, cruisingTurnGain, searchingPower,
                            start2FireDistance, state,state+1);
                }

                if (state == 1) {
                    // prepare to shoot
                    robot.motorLeftWheel.setPower(0.0);
                    robot.motorRightWheel.setPower(0.0);
                    particleShooter.start(0);
                    particleShooter.armPower = leftArmAutoMovePower;
                    particleShooter.armStartPosition = leftArmFiringSafeZone;
                }
                break;
            case 1:
                // shoot particles
                state = particleShooter.loop(state, state+1);
                break;
            case 2:
                // turn 45 degree
                gyroTracker.skewTolerance = 3;
                state = gyroTracker.turn(fire2TurnDegree, inPlaceTurnGain,
                        turningPower,state,state+1);

                if (state == 3) {
                    // activate jamming detection
                    jammingDetection.reset();
                }
                break;
            case 3:
                // go straight until hit the wall
                gyroTracker.skewTolerance = 0;
                gyroTracker.breakDistance = 0;
                state = gyroTracker.goStraight (fire2TurnDegree, cruisingTurnGain,
                        cruisingPower, fire2WallDistance, state,state+2); // need to +2 to skip jam backup

                // jamming detection
                if (jammingDetection.isJammed(Math.min(robot.motorLeftWheel.getCurrentPosition(),
                        robot.motorRightWheel.getCurrentPosition()))) {
                    gyroTracker.minTurnPower = 0.01;
                    stopWheels();
                    gyroTracker.setWheelLandmark(); // important. otherwise it use last landmark
                    state = 4;
                }
                break;
            case 4:
                // if jammed, back up a little bit
                gyroTracker.breakDistance = 0;
                state = gyroTracker.goStraight (fire2TurnDegree, cruisingTurnGain,
                        -1.0*searchingPower, jammingBackupDistance, state,state+1);
                break;
            case 5:
                // turn -45 degree back
                gyroTracker.skewTolerance = 1;
                gyroTracker.maxTurnPower = 0.2;
                state = gyroTracker.turn(fire2TurnDegree+wall2TurnDegree,
                        inPlaceTurnGain,turningPower,state,state+1);
                if (state == 6 ) {
                    // reset min turning power to avoid jerky movements
                    gyroTracker.minTurnPower = 0.01;
                }
                break;
            case 6:
                // go straight until hit first white line
                gyroTracker.skewTolerance = 0;
                gyroTracker.breakDistance = 200;
                state = gyroTracker.goStraight (fire2TurnDegree+wall2TurnDegree,
                        cruisingTurnGain, searchingPower, wall2BeaconDistance, state,state+1);

                // check the ods for white line signal
                if (hardwareLineTracker.onWhiteLine(groundBrightness, 2)) {
                    state = 7;
                    gyroTracker.setWheelLandmark();
                    stopWheels();
                    beaconPresser.start(0);
                }
                break;
            case 7:
                // touch beacon
                state = beaconPresser.loop(state, state+1);
                if (state == 8) {
                    gyroTracker.setWheelLandmark();
                    lastTimeStamp = System.currentTimeMillis();
                }
                break;
            case 8:
                // go straight until hit the second white line
                gyroTracker.skewTolerance = 0;
                gyroTracker.breakDistance = 200;
                if (System.currentTimeMillis() - lastTimeStamp > 800) {
                    state = gyroTracker.goStraight(fire2TurnDegree + wall2TurnDegree,
                            cruisingTurnGain, searchingPower, beacon2BeaconDistance, state, state + 1);
                } else {
                    state = gyroTracker.goStraight(fire2TurnDegree + wall2TurnDegree,
                            cruisingTurnGain, cruisingPower, beacon2BeaconDistance, state, state + 1);
                }

                // check the ods for white line signal
                if (gyroTracker.getWheelLandmarkOdometer() > 1000
                        && hardwareLineTracker.onWhiteLine(groundBrightness, 2)) {
                    state = 9;
                    stopWheels();
                    gyroTracker.setWheelLandmark();
                    beaconPresser.start(0);
                }
                break;
            case 9:
                // touch beacon
                state = beaconPresser.loop(state, state+1);

                if (state == 10) {
                    gyroTracker.setWheelLandmark();
                }
                break;
            case 10:
                // backup straight to central parking
                gyroTracker.skewTolerance = 0;
                state = gyroTracker.goStraight (fire2TurnDegree+wall2TurnDegree+beacon2ParkTurnDegree,
                        cruisingTurnGain, -1*chargingPower, beacon2ParkingDistance, state,state+1);

                if (System.currentTimeMillis() - lastTimeStamp > 500) {
                    VortexUtils.moveMotorByEncoder(robot.motorLeftArm,
                            leftArmMovePosition, leftArmAutoMovePower);
                }

                break;
            default:
                // stop
                telemetry.addData("State:", "End");
                stop();
        }
        telemetry.update();
    }
}
