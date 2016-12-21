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

import com.qualcomm.robotcore.eventloop.SyncdDevice;
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

@Autonomous(name="Dance: Eye of the Tiger", group="Dance")
public class DanceAutoOp extends VortexAutoOp{

    protected int headPositionA = 2500;
    protected int headPositionB = 2800;
    protected int headPositionC = 3000;

    @Override
    public void start() {
        super.start();
        particleShooter.cock();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("State:", "%02d", state);
        switch (state) {
            case 0:
                state = cowboyDance(555);
                if(state == 1) lastTimeStamp = System.currentTimeMillis();
                break;
            case 1:
                state = cowboyDance2(555);
                if(state == 2) lastTimeStamp = System.currentTimeMillis();
                break;
            case 2:
                state = cowboyDance3(555);
                if(state == 3) lastTimeStamp = System.currentTimeMillis();
                break;
            default:
                lastTimeStamp = System.currentTimeMillis();
                break;
        }
    }

    public void findEmptySpot () {
        // make circle to collect data

        // move to empty spot
    }

    public int cowboyDance (int beatInterval) {
        switch (state) {
            case 0:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval) {
                    armB();
                    headA(0.3);
                } else {
                    state = 1;
                }
                break;
            case 1:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*2) {
                    armC();
                } else {
                    state = 2;
                }
                break;
            case 2:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*3) {
                    headC(0.3);
                    armD();
                } else {
                    state = 3;
                }
                break;
            case 3:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*4) {
                    armE();
                } else {
                    state = 4;
                }
                break;
            case 4:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*5) {
                    armF();
                    headB(0.3);
                } else {
                    state = 5;
                }
                break;
            case 5:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*6) {
                    armG();
                    headC(0.3);
                } else {
                    state = 6;
                }
                break;
            case 6:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*7) {
                    armH();
                    headB(0.3);
                } else {
                    state = 7;
                }
                break;
            case 7:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*8) {
                    armA();
                    headC(0.3);
                } else {
                    state = 8;
                }
                break;
            default:
                return 1; // done
        }
        return 0;
    }

    public int cowboyDance2 (int beatInterval) {
        switch (state) {
            case 0:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval) {
                    armB();
                    headA(0.3);
                    wheelB(0.0,15);
                } else {
                    state = 1;
                }
                break;
            case 1:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*2) {
                    armC();
                } else {
                    state = 2;
                }
                break;
            case 2:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*3) {
                    headC(0.3);
                    armD();
                    wheelB(0.0,-15);
                } else {
                    state = 3;
                }
                break;
            case 3:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*4) {
                    armE();
                } else {
                    state = 4;
                }
                break;
            case 4:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*5) {
                    armC();
                    headB(0.3);
                    wheelB(0.0,15);
                } else {
                    state = 5;
                }
                break;
            case 5:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*6) {
                    armD();
                    headC(0.3);
                    wheelB(0.0,-15);
                } else {
                    state = 6;
                }
                break;
            case 6:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*7) {
                    armC();
                    headB(0.3);
                    wheelB(0.0,15);
                } else {
                    state = 7;
                }
                break;
            case 7:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*8) {
                    armD();
                    headC(0.3);
                    wheelB(0.0,-15);
                } else {
                    state = 8;
                }
                break;
            default:
                return 2; // done
        }
        return 1;
    }

    public int cowboyDance3 (int beatInterval) {
        switch (state) {
            case 0:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval) {
                    armB();
                    headA(0.3);
                    wheelB(0.0,15);
                } else {
                    state = 1;
                }
                break;
            case 1:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*2) {
                    armC();
                    wheelB(0.0,50);
                } else {
                    state = 2;
                }
                break;
            case 2:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*3) {
                    headC(0.3);
                    armD();
                    wheelB(0.0,90);
                } else {
                    state = 3;
                }
                break;
            case 3:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*4) {
                    armE();
                    wheelB(0.0,135);
                } else {
                    state = 4;
                }
                break;
            case 4:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*5) {
                    armG();
                    headB(0.3);
                    wheelB(0.0,180);
                } else {
                    state = 5;
                }
                break;
            case 5:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*6) {
                    armH();
                    headC(0.3);
                    wheelB(0.0,240);
                } else {
                    state = 6;
                }
                break;
            case 6:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*7) {
                    armG();
                    headB(0.3);
                    wheelB(0.0,290);
                } else {
                    state = 7;
                }
                break;
            case 7:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*8) {
                    armH();
                    headC(0.3);
                    wheelB(0.0,0);
                } else {
                    state = 8;
                }
                break;
            default:
                return 3; // done
        }
        return 2;
    }

    // arm dance modes
    public void armA () {
        leftBeaconArm.retract();
        rightBeaconArm.retract();
    }

    public void armB () {
        leftBeaconArm.extendUntilTouch(0.8);
        rightBeaconArm.extendUntilTouch(0.8);
    }

    public void armC () {
        leftBeaconArm.extendUntilTouch(0.8);
        rightBeaconArm.retract();
    }

    public void armD () {
        leftBeaconArm.retract();
        rightBeaconArm.extendUntilTouch(0.8);
    }

    public void armE () {
        leftBeaconArm.raiseUpperArm();
        rightBeaconArm.raiseUpperArm();
    }

    public void armF () {
        leftBeaconArm.lowerLowerArm();
        rightBeaconArm.lowerLowerArm();
    }

    public void armG () {
        leftBeaconArm.raiseUpperArm();
        rightBeaconArm.lowerLowerArm();
    }

    public void armH () {
        leftBeaconArm.lowerLowerArm();
        rightBeaconArm.raiseUpperArm();
    }

    // wheel dance modes
    public void  wheelA (double power, int distance) {
        gyroTracker.skewTolerance = 0;
        gyroTracker.goStraight (0, cruisingTurnGain, power,
                distance, 0,1);
    }

    public void wheelB (double turnPower, int heading) {
        gyroTracker.skewTolerance = 1;
        gyroTracker.maxTurnPower = 0.2;
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
