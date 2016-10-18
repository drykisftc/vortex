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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

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

    BeaconToucher beaconToucher = null;
    ParticleShooter particleShooter = null;

    int beacon2ParkTurnDegree = -135;

    // to do: add wall tracker

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        super.init();
        beaconToucher = new BeaconToucher();
        beaconToucher.setReporter(telemetry);
        particleShooter = new ParticleShooter(robot.motorLeftArm, robot.motorLeftHand);
        particleShooter.setReporter(telemetry);
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
        beaconToucher.start(0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (state) {
            case 0:
                // go straight
                state = goStraight (landMarkAngle, cruisingTurnSensitivity, cruisingPower,
                        landMarkPosition, start2FireDistance, state,state+1);
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
                state = turn(landMarkAngle+fire2TurnDegree, inplaceTurnSensitivity,
                        turningPower,state,state+1);
                telemetry.addData("State:", "%02d", state);
                break;
            case 3:
                // go straight until hit wall
                state = goStraight (landMarkAngle+fire2TurnDegree, cruisingTurnSensitivity,
                        cruisingPower, landMarkPosition, fire2WallDistance, state,state+1);
                telemetry.addData("State:", "%02d", state);
                break;
            case 4:
                // turn -45 degree back
                state = turn(landMarkAngle+fire2TurnDegree+wall2TurnDegree,
                        inplaceTurnSensitivity,turningPower,state,state+1);
                telemetry.addData("State:", "%02d", state);
                break;
            case 5:
                // go straight until hit first white line
                state = goStraight (landMarkAngle+fire2TurnDegree+wall2TurnDegree,
                        cruisingTurnSensitivity, cruisingPower, landMarkPosition, wall2BeaconDistance, state,state+1);
                telemetry.addData("State:", "%02d", state);
                if (state == 6) {
                    beaconToucher.start(0);
                }
                break;
            case 6:
                // touch beacon
                state = beaconToucher.loop(state, state+1);
                break;
            case 7:
                // go straight until hit the second white line
                if ( state == 8 ) {
                    beaconToucher.start(0);
                }
                break;
            case 8:
                // touch beacon
                state = beaconToucher.loop(state, state+1);
                break;
            case 9:
                // turn 135 degree
                state = goStraight (landMarkAngle+fire2TurnDegree+wall2TurnDegree+beacon2ParkTurnDegree,
                        cruisingTurnSensitivity, cruisingPower, landMarkPosition, wall2BeaconDistance, state,state+1);
                telemetry.addData("State:", "%02d", state);
                break;
            case 10:
                // go straight to central parking
                break;
            default:
                // stop
                telemetry.addData("State:", "End");
                stop();
        }
        telemetry.update();
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
