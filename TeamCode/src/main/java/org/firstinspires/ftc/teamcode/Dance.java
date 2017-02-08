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

@Autonomous(name="Dance: Eye of Tiger", group="zDance")
public class Dance extends VortexAutoOp{
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

    protected void dancePatternReset () {
        danceState = 0;
        lastTimeStamp = System.currentTimeMillis();
    }
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
                state = cowboyDance(danceBeats, 2, 3);
                if(state == 3) dancePatternReset();
                break;
            case 3:
                state = cowboyDance2(danceBeats, 3, 4);
                if(state == 4) dancePatternReset();
                break;
            case 4:
                state = cowboyDance3(danceBeats, 4,5);
                if(state == 5) dancePatternReset();
                break;
            case 5:
                state = looper(danceBeats,5,2); // repeat to 2
                if(state == 2) dancePatternReset();
                break;
            default:
                dancePatternReset();
                break;
        }
    }

    public void findEmptySpot () {
        // make circle to collect data

        // move to empty spot
    }
    public int looper (int beatInterval, int startState, int endState) {
        telemetry.addData("looper State:", "%02d", danceState);
        switch (danceState) {
            case 0:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval) {
                    armC(5.0);
                } else {
                    danceState = 1;
                }
                break;
            case 1:

                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*2) {
                    armD(5.0);
                } else {
                    danceState = 2;
                }
                break;
            case 2:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*5) {
                    armC(5.0);
                } else {
                    danceState = 3;
                }
                break;
            case 3:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*6) {
                    armD(5.0);
                } else {
                    danceState = 4;
                }
                break;
            case 4:

                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*7) {
                    armC(5.0);
                } else {
                    danceState = 5;
                }
                break;
            case 5:

                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*10) {
                    armD(5.0);
                } else {
                    danceState = 6;
                }
                break;
            default:
                return endState; // done
        }
        telemetry.addData("back to", "state", "0");
        return startState;
    }
    public int beforeShootDance (int beatInterval, int startState, int endState) {
        telemetry.addData("Pre-shoot State:", "%02d", danceState);
        switch (danceState) {
            case 0:
                //delay
                if (System.currentTimeMillis() - lastTimeStamp < 0) {

                }else{
                    danceState = 1;
                }
            case 1:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval) {
                    armC(5.0);
                } else {
                    danceState = 2;
                }
                break;
            case 2:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*3) {
                    armD(5.0);
                } else {
                    danceState = 3;
                }
                break;
            case 3:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*4) {
                    armC(5.0);
                } else {
                    danceState = 4;
                }
                break;
            case 4:

                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*5) {
                    armD(5.0);
                } else {
                    danceState = 5;
                }
                break;
            case 5:

                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*7) {
                    armC(5.0);
                } else {
                    danceState = 6;
                }
                break;
            case 6:

                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*8) {
                    armD(5.0);
                } else {
                    danceState = 7;
                }
                break;
            case 7:

                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*9) {
                    armC(5.0);
                } else {
                    danceState = 8;
                }
                break;
            case 8:

                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*11) {
                    armD(5.0);
                } else {
                    danceState = 9;
                }
                break;
            default:
                return endState; // done
        }
        return startState;
    }

    public int cowboyDance (int beatInterval, int startState, int endState) {
        telemetry.addData("Cowboy Dance State:", "%02d", danceState);
        switch (danceState) {
            case 0:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval) {
                    armA();
                    headA(headPower);
                } else {
                    danceState = 1;

                }
                break;
            case 1:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*2) {
                    armB(5.0);

                } else {
                    danceState = 2;
                }
                break;
            case 2:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*5) {
                    headC(headPower);
                    armA();
                } else {
                    danceState = 3;
                }
                break;
            case 3:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*7) {
                    armG();
                } else {
                    danceState = 4;
                }
                break;
            case 4:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*8) {
                    armB(5.0);
                    headB(headPower);
                } else {
                    danceState = 5;
                }
                break;
            case 5:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*9) {
                    armC(5.0);
                    headC(headPower);
                } else {
                    danceState = 6;
                }
                break;
            case 6:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*11) {
                    armH();
                    headB(headPower);
                } else {
                    danceState = 7;
                }
                break;
            case 7:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*12) {
                    armA();
                    headC(headPower);
                } else {
                    danceState = 8;
                }
                break;
            default:
                return endState; // done
        }
        return startState;
    }

    public int cowboyDance2 (int beatInterval, int startState, int endState) {
        telemetry.addData("Cowboy Dance2 State:", "%02d", danceState);
        switch (danceState) {
            case 0:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval) {
                    armB(5.0);
                    headA(headPower);
                    wheelB(0.0,15);
                } else {
                    danceState = 1;
                }
                break;
            case 1:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*3) {
                    armC(5.0);
                } else {
                    danceState = 2;
                }
                break;
            case 2:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*4) {
                    headC(headPower);
                    armD(5.0);
                    wheelB(0.0,-15);
                } else {
                    danceState = 3;
                }
                break;
            case 3:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*5) {
                    armE();
                } else {
                    danceState = 4;
                }
                break;
            case 4:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*8) {
                    armC(5.0);
                    headB(headPower);
                    wheelB(0.0,15);
                } else {
                    danceState = 5;
                }
                break;
            case 5:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*10) {
                    armD(5.0);
                    headC(headPower);
                    wheelB(0.0,-15);
                } else {
                    danceState = 6;
                }
                break;
            case 6:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*11) {
                    armC(5.0);
                    headB(headPower);
                    wheelB(0.0,15);
                } else {
                    danceState = 7;
                }
                break;
            case 7:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*12) {
                    armD(5.0);
                    headC(headPower);
                    wheelB(0.0,-15);
                } else {
                    danceState = 8;
                }
                break;
            default:
                return endState; // done
        }
        return startState;
    }

    public int cowboyDance3 (int beatInterval, int startState, int endState) {
        telemetry.addData("Cowboy Dance3 State:", "%02d", danceState);
        switch (danceState) {
            case 0:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval) {
                    armB(5.0);
                    headA(headPower);
                    wheelB(0.0,15);
                } else {
                    danceState = 1;
                }
                break;
            case 1:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*2) {
                    armC(5.0);
                    wheelB(0.0,50);
                } else {
                    danceState = 2;
                }
                break;
            case 2:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*3) {
                    headC(headPower);
                    armD(5.0);
                    wheelB(0.0,90);
                } else {
                    danceState = 3;
                }
                break;
            case 3:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*5) {
                    armE();
                    wheelB(0.0,135);
                } else {
                    danceState = 4;
                }
                break;
            case 4:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*6) {
                    armG();
                    headB(headPower);
                    wheelB(0.0,180);
                } else {
                    danceState = 5;
                }
                break;
            case 5:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*7) {
                    armH();
                    headC(headPower);
                    wheelB(0.0,240);
                } else {
                    danceState = 6;
                }
                break;
            case 6:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*10) {
                    armG();
                    headB(headPower);
                    wheelB(0.0,290);
                } else {
                    danceState = 7;
                }
                break;
            case 7:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*12) {
                    armH();
                    headC(headPower);
                    wheelB(0.0,0);
                } else {
                    danceState = 8;
                }
                break;
            default:
                return endState; // done
        }
        return startState;
    }
    // arm dance modes
    /*
    *********************************************************************************************
     */
    public void armA () {
        leftBeaconArm.retract();
        rightBeaconArm.retract();
    }

    public void armB (double speed) {
        leftBeaconArm.extend(speed);
        rightBeaconArm.extend(speed);
    }

    public void armC (double speed) {
        leftBeaconArm.extend(speed);
        rightBeaconArm.retract();
    }

    public void armD (double speed) {
        leftBeaconArm.retract();
        rightBeaconArm.extend(speed);
    }

    public void armE () {
        leftBeaconArm.raiseUpperArm();
        leftBeaconArm.raiseLowerArm();
        rightBeaconArm.raiseUpperArm();
        rightBeaconArm.raiseLowerArm();
    }

    public void armF () {
        leftBeaconArm.raiseUpperArm();
        leftBeaconArm.raiseLowerArm();
        rightBeaconArm.raiseUpperArm();
        rightBeaconArm.raiseLowerArm();
    }

    public void armG () {
        leftBeaconArm.raiseUpperArm();
        rightBeaconArm.lowerLowerArm();
        leftBeaconArm.raiseLowerArm();
        rightBeaconArm.raiseLowerArm();
    }

    public void armH () {
        leftBeaconArm.lowerLowerArm();
        leftBeaconArm.lowerUpperArm();
        rightBeaconArm.raiseUpperArm();
        rightBeaconArm.raiseLowerArm();
    }

    // wheel dance modes
    public void  wheelA (double power, int distance) {
        gyroTracker.goStraight (0, cruisingTurnGain, power,
                distance, 0,1);
    }

    public void wheelB (double turnPower, int heading) {
        gyroTracker.maxTurnPower = 0.15;
        state = gyroTracker.turn(heading,
                inPlaceTurnGain,turnPower,0,0);
    }

    // head dance mode
    public void headA ( double power) {
        VortexUtils.moveMotorByEncoder(robot.motorLeftArm, headPositionA, power);
    }

    public void headB ( double power) {
        VortexUtils.moveMotorByEncoder(robot.motorLeftArm, headPositionB, power);
    }

    public void headC ( double power) {
        VortexUtils.moveMotorByEncoder(robot.motorLeftArm, headPositionC, power);
    }
}
