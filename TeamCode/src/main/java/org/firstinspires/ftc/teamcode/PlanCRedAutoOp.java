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

@Autonomous(name="Plan C: Red", group="Plan C")
public class PlanCRedAutoOp extends VortexAutoOp{

    @Override
    public void start() {
        super.start();
        start2FireDistance = 2900; //2500
        fire2TurnDegree = -150;
        fire2WallDistance= -2400; // go backwards
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (state) {
            case 0:
                if (System.currentTimeMillis() - lastTimeStamp > 5000) {
                    state = 1;
                }
                break;
            case 1:
                // go straight
                state = gyroTracker.goStraight(0, cruisingTurnGain, cruisingPower,
                        start2FireDistance, state, state + 1);
                telemetry.addData("State:", "%02d", state);

                if (System.currentTimeMillis() - lastTimeStamp > 200) {
                    VortexUtils.moveMotorByEncoder(robot.motorLeftArm,
                            leftArmFirePosition, leftArmAutoMovePower);
                }

                if (state == 2) {
                    // prepare to shoot
                    robot.motorLeftWheel.setPower(0.0);
                    robot.motorRightWheel.setPower(0.0);
                    particleShooter.start(0);
                    particleShooter.armPower = leftArmAutoMovePower;
                    particleShooter.armStartPosition = leftArmFiringSafeZone;
                }
                break;
            case 2:
                // shoot particles
                state = particleShooter.loop(state, state + 1);
                break;
            case 3:
                // turn 45 degree
                gyroTracker.skewTolerance = 3;
                state = gyroTracker.turn(fire2TurnDegree, inPlaceTurnGain,
                        turningPower,state,state+1);
                break;
            case 4:
                // go backward and park
                gyroTracker.skewTolerance = 0;
                gyroTracker.breakDistance = 0;
                state = gyroTracker.goStraight (fire2TurnDegree, cruisingTurnGain,
                        -1.0*cruisingPower, fire2WallDistance, state,state+1);
                break;
            default:
                // stop
                telemetry.addData("State:", "End");
                stop();
        }
        telemetry.update();
    }
}
