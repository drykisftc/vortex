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

@Autonomous(name="DanceCircle", group="zDance")
public class DanceCicle extends Dance{
    protected int headPositionA = 2500;
    protected int headPositionB = 2800;
    protected int headPositionC = 3000;
    int danceState = 0;
    int danceBeats = 556;
    double headPower = 0.4;
    double armSpeed = 2.0;


    @Override
    public void start() {
        super.start();
        particleShooter.cock();
        lastTimeStamp = System.currentTimeMillis();
        danceState = 0;
        state = 0;

    }
/*
    protected void dancePatternReset () {
        danceState = 0;
        armA();
        lastTimeStamp = System.currentTimeMillis();
    }
    */

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("State:", "%02d", state);
        telemetry.addData("DanceState:", "%02d", danceState);
        telemetry.addData("Current Time: ", System.currentTimeMillis() - lastTimeStamp);
        switch (state) {
            case 0:
                gyroTracker.skewTolerance = 0;
                if(gyroTracker.goStraight (0, cruisingTurnGain, 0.2,
                    start2FireDistance, 0, 1) == 1){
                    start2FireDistance = 0;
                }

                VortexUtils.moveMotorByEncoder(robot.motorLeftArm,
                        leftArmFirePosition, 0.25);
                armB(0.2);

                if (System.currentTimeMillis() - lastTimeStamp > 8200) {
                    state = 1;
                    lastTimeStamp = System.currentTimeMillis();
                }
                break;
            case 1:
                state = beforeShootDance(danceBeats, 1, 2);
                if(state == 2) dancePatternReset();
                break;
            case 2:
                cowboyDance(danceBeats, 2, 3);

                state = gyroTracker.goStraight (0, cruisingTurnGain,
                        cruisingPower, testDistance1-3200, state, -1);
                if(state == -1) dancePatternReset();
                break;
            case -1:// turn 90 degrees
                gyroTracker.skewTolerance = 2;
                cowboyDance3(danceBeats, 4,5);
                state = gyroTracker.turn(testTurnAngle1, inPlaceTurnGain,turningPower,state, 3);
                if(state == 3)dancePatternReset();
                break;
            case 3:
                cowboyDance2(danceBeats, 3, 4);
                gyroTracker.skewTolerance = 0;
                state = gyroTracker.goStraight (testTurnAngle1, cruisingTurnGain,
                        searchingPower, testDistance1-100,state, -2);

                if(state == -2) dancePatternReset();
                break;
            case -2: // turn 180 degrees
                cowboyDance(danceBeats, 4,5);
                gyroTracker.skewTolerance = 2;
                state = gyroTracker.turn(testTurnAngle1, inPlaceTurnGain,turningPower,state, 4);
                if (state == 4)dancePatternReset();
                break;
            case 4:
                cowboyDance3(danceBeats, 4,5);
                gyroTracker.skewTolerance = 0;
                state = gyroTracker.goStraight (testTurnAngle1*2, cruisingTurnGain,
                        cruisingPower, testDistance1-6000, state, -3);
                if(state == -3) dancePatternReset();
                break;
            case -3:
                // backup
                /*gyroTracker.skewTolerance = 0;
                state = gyroTracker.goStraight (testTurnAngle1*3, cruisingTurnGain,
                        -1.0*cruisingPower, backDistance-100, state, -4);
                */
                //break;
            case -4:
                // turn -90 degrees
                cowboyDance(danceBeats, 4,5);
                gyroTracker.skewTolerance = 2;
                state = gyroTracker.turn(testTurnAngle1*2, inPlaceTurnGain,turningPower,state, 5);
                if (state == 5)dancePatternReset();
                break;
            case 5:
                gyroTracker.skewTolerance = 0;
                state = gyroTracker.goStraight (testTurnAngle1*2, cruisingTurnGain,
                        cruisingPower, testDistance1-100, state, state+1);
                looper(danceBeats,5,2);

                if(state == 6) dancePatternReset();
                break;
            case 6:
                state = gyroTracker.turn(-110, inPlaceTurnGain,turningPower,state, 7);
                cowboyDance2(danceBeats, 4,5);
                if(state == 7)dancePatternReset();
                break;
            case 7:
                state = gyroTracker.goStraight (testTurnAngle1*-1, cruisingTurnGain,
                        cruisingPower, testDistance1-100, state, state+1);
            default:
                dancePatternReset();
                break;
        }
    }



    // wheel dance modes
    @Override
    public void  wheelA (double power, int distance) {
    //    gyroTracker.skewTolerance = 0;
    //    gyroTracker.goStraight (0, cruisingTurnGain, power,
    //            distance, 0,1);
    }
    @Override
    public void wheelB (double turnPower, int heading) {
    /*    gyroTracker.skewTolerance = 1;
        gyroTracker.maxTurnPower = 0.2;
        state = gyroTracker.turn(heading,
                inPlaceTurnGain,turnPower,0,0);
                */
    }
}