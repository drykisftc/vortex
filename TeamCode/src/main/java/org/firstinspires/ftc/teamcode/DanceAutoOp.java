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

import com.qualcomm.hardware.adafruit.BNO055IMU;
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

@Autonomous(name="DanceAutoOp", group="zDance")
public class DanceAutoOp extends VortexAutoOp{

    protected int headPositionA = 2500;
    protected int headPositionB = 2800;
    protected int headPositionC = 3000;
    int danceState = 0;
    int danceBeats = 556;
    double headPower = 0.4;
    double armSpeed = 2.0;
    @Override
    public void init(){
        super.init();
        jammingDetection = new JammingDetection (1000L);
    }
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
        telemetry.addData("Current Time: ", "%02d", System.currentTimeMillis() - lastTimeStamp);
        switch (state) {
            case 0:
                gyroTracker.skewTolerance = 0;
                if(gyroTracker.goStraight (0, cruisingTurnGain, 0.2,
                        start2FireDistance, 0, 1) == 1){
                    start2FireDistance = 0;
                }
                VortexUtils.moveMotorByEncoder(robot.motorLeftArm,
                        leftArmFirePosition, 0.25);
                if(System.currentTimeMillis() - lastTimeStamp < 7000){
                    armB(0.2);
                } else {
                    armA();
                }
                if (System.currentTimeMillis() - lastTimeStamp > 8000) {
                    state = 1;
                    lastTimeStamp = System.currentTimeMillis();
                }
                break;
            case 1:
                state = beforeShootDance(danceBeats, 1, 2);
                if(state == 2) {
                    dancePatternReset();
                    particleShooter.start(0);
                    particleShooter.armPower = leftArmAutoMovePower;
                    particleShooter.armStartPosition = leftArmFiringSafeZone;
                }
                break;
            case 2:
                //shoot
                particleShooter.loop(state, state+1);
                state = cowboyDance(danceBeats, 2, 4);
                if(state == 4) dancePatternReset();
                break;
            case 3:
            case 4:
                gyroTracker.skewTolerance = 1;
                gyroTracker.turn(fire2TurnDegree, inPlaceTurnGain,
                        turningPower,state,state+1);
                state = cowboyDance2(danceBeats, 4, 5);
                if(state == 5) dancePatternReset();

                if (state == 5) {
                // activate jamming detection
                jammingDetection.reset();
                }
                break;
            case 5:
                wallTracker.readDistance();

                // go straight until hit the wall
                gyroTracker.skewTolerance = 0;
                gyroTracker.breakDistance = 800;
                state = gyroTracker.goStraight (fire2TurnDegree, cruisingTurnGain,
                        cruisingPower, fire2WallDistance, state,state+2); // need to +2 to skip jam backup

                double sonicDistance = wallTracker.getHistoryDistanceAverage();
                telemetry.addData("Wall Distance: ", "%02f", sonicDistance);
                int travelDistance = Math.min(robot.motorLeftWheel.getCurrentPosition(),
                        robot.motorRightWheel.getCurrentPosition());
                telemetry.addData("Travel Distance: ", travelDistance);
                // wall distance detection
                if ( (Math.abs(travelDistance - gyroTracker.getWheelLandmark()) > fire2WallDistance * 0.6
                        && sonicDistance <= sonicWallDistanceLimit))  {
                    stopWheels();
                    gyroTracker.setWheelLandmark();
                    state = 7;
                }

                // jamming detection
                if (jammingDetection.isJammed(travelDistance)) {
                    gyroTracker.minTurnPower = 0.01;
                    gyroTracker.breakDistance = 100;
                    stopWheels();
                    gyroTracker.setWheelLandmark();
                    state = 6;
                }
                cowboyDance3(danceBeats, 5,6);
                if(state == 6||state == 7) dancePatternReset();
            case 6:
                // if jammed, back up a little bit
                gyroTracker.breakDistance = 0;
                state = gyroTracker.goStraight (fire2TurnDegree, cruisingTurnGain,
                        -1.0*searchingPower, jammingBackupDistance, state,state+1);
                break;
            case 7:
                // turn -45 degree back
                gyroTracker.skewTolerance = 0;
                gyroTracker.maxTurnPower = 0.2;
                state = gyroTracker.turn(fire2TurnDegree+wall2TurnDegree,
                        inPlaceTurnGain,turningPower,state,state+1);
                if (state == 8 ) {
                    // reset min turning power to avoid jerky movements
                    gyroTracker.minTurnPower = 0.01;
                }
                break;
            case 8:
                // go straight until hit first white line
                gyroTracker.skewTolerance = 0;
                gyroTracker.breakDistance = 200;
                state = gyroTracker.goStraight (fire2TurnDegree+wall2TurnDegree,
                        cruisingTurnGain, searchingPower, wall2BeaconDistance, state,state+1);

                // check the ods for white line signal

                if (hardwareLineTracker.onWhiteLine(groundBrightness, 2)) {
                    state = 7;
                    stopWheels();
                    gyroTracker.setWheelLandmark();
                    beaconPresser.start(0);
                }
                break;
            case 9:
                // touch beacon
                state = beaconPresser.loop(state, state+1);
                if (state == 8) {
                    gyroTracker.setWheelLandmark();
                    lastTimeStamp = System.currentTimeMillis();
                }
            case 10:
                // go straight until hit the second white line
                gyroTracker.skewTolerance = 0;
                gyroTracker.breakDistance = 200;
                if (System.currentTimeMillis() - lastTimeStamp > 1500) {
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
            case 11:
                // touch beacon
                state = beaconPresser.loop(state, state+1);

                if (state == 10) {
                    gyroTracker.setWheelLandmark();
                }
                break;
            case 12:
                // turn 45 degree
                gyroTracker.skewTolerance = 1;
                state = gyroTracker.turn(fire2TurnDegree+wall2TurnDegree+beacon2ParkTurnDegree,
                        inPlaceTurnGain,parkTurningPower,state,state+1);
                if (state == 11) {
                    lastTimeStamp = System.currentTimeMillis();
                    gyroTracker.minTurnPower = 0.01;
                }


            default:
                dancePatternReset();
                //state = 0; // repeat
                break;
        }
    }

    public void findEmptySpot () {
        // make circle to collect data

        // move to empty spot
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
        telemetry.addData("back to", "state", "0");
        return startState;
    }

    public int cowboyDance (int beatInterval, int startState, int endState) {
        telemetry.addData("Cowboy Dance State:", "%02d", danceState);
        switch (danceState) {
            case 0:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval) {
                    armA();
                    //headA(headPower);
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
                    //headC(headPower);
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
                    //headB(headPower);
                } else {
                    lastTimeStamp+= 556*0;
                    danceState = 5;
                }
                break;
            case 5:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*9) {
                    armC(5.0);
                    //headC(headPower);
                } else {
                    danceState = 6;
                }
                break;
            case 6:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*11) {
                    armH();
                    //headB(headPower);
                } else {
                    danceState = 7;
                }
                break;
            case 7:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*12) {
                    armA();
                    //headC(headPower);
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
                    //wheelB(0.0,15);
                } else {
                    danceState = 1;
                }
                break;
            case 1:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*2) {
                    armC(5.0);
                } else {
                    danceState = 2;
                }
                break;
            case 2:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*3) {
                    headC(headPower);
                    armD(5.0);
                    //wheelB(0.0,-15);
                } else {
                    danceState = 3;
                }
                break;
            case 3:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*4) {
                    armE();
                } else {
                    danceState = 4;
                }
                break;
            case 4:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*5) {
                    armC(5.0);
                    headB(headPower);
                    //wheelB(0.0,15);
                } else {
                    danceState = 5;
                }
                break;
            case 5:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*6) {
                    armD(5.0);
                    headC(headPower);
                    //wheelB(0.0,-15);
                } else {
                    danceState = 6;
                }
                break;
            case 6:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*7) {
                    armC(5.0);
                    headB(headPower);
                    //wheelB(0.0,15);
                } else {
                    danceState = 7;
                }
                break;
            case 7:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*8) {
                    armD(5.0);
                    headC(headPower);
                    //wheelB(0.0,-15);
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
                    //wheelB(0.0,15);
                } else {
                    danceState = 1;
                }
                break;
            case 1:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*2) {
                    armC(5.0);
                    //wheelB(0.0,50);
                } else {
                    danceState = 2;
                }
                break;
            case 2:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*3) {
                    headC(headPower);
                    armD(5.0);
                    //wheelB(0.0,90);
                } else {
                    danceState = 3;
                }
                break;
            case 3:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*4) {
                    armE();
                    //wheelB(0.0,135);
                } else {
                    danceState = 4;
                }
                break;
            case 4:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*5) {
                    armG();
                    headB(headPower);
                    //wheelB(0.0,180);
                } else {
                    danceState = 5;
                }
                break;
            case 5:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*6) {
                    armH();
                    headC(headPower);
                    //wheelB(0.0,240);
                } else {
                    danceState = 6;
                }
                break;
            case 6:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*7) {
                    armG();
                    headB(headPower);
                    //wheelB(0.0,290);
                } else {
                    danceState = 7;
                }
                break;
            case 7:
                if (System.currentTimeMillis() - lastTimeStamp < beatInterval*8) {
                    armH();
                    headC(headPower);
                    //wheelB(0.0,0);
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
        leftBeaconArm.extendUntilTouch(speed);
        rightBeaconArm.extendUntilTouch(speed);
    }

    public void armC (double speed) {
        leftBeaconArm.extendUntilTouch(speed);
        rightBeaconArm.retract();
    }

    public void armD (double speed) {
        leftBeaconArm.retract();
        rightBeaconArm.extendUntilTouch(speed);
    }

    public void armE () {
        leftBeaconArm.raiseUpperArm();
        leftBeaconArm.raiseLowerArm();
        rightBeaconArm.raiseUpperArm();
        rightBeaconArm.raiseLowerArm();
    }

    public void armF () {
        leftBeaconArm.lowerUpperArm();
        leftBeaconArm.raiseLowerArm();
        rightBeaconArm.lowerUpperArm();
        rightBeaconArm.raiseLowerArm();
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
