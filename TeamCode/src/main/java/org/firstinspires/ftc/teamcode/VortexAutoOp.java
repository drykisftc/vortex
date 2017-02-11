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

@Autonomous(name="Auto: base", group="Plan A")
@Disabled
public class VortexAutoOp extends GyroTrackerOpMode{

    protected BeaconPresser beaconPresser = null;
    protected HardwareLineTracker hardwareLineTracker = null;
    protected WallTracker wallTracker = null;

    protected double groundBrightness = 0.0;
    protected double minLineBrightness = 0.02;

    // navigation settings
    protected int start2FireDistance = 2500; //2500
    protected int fire2TurnDegree = 75;
    protected int fire2WallDistance = 4600; // 5121
    protected int wall2TurnDegree = -75;
    protected int wall2BeaconDistance = 800; //953 actually
    protected int beacon2ParkTurnDegree = 45;
    protected int beacon2BeaconDistance = 4800; //4325
    protected int beacon2PickBallDistance = 400; //4318
    protected int beacon2ParkingDistance = 5000; //4318
    protected int jammingBackupDistance = 150;
    protected double sonicWallDistanceLimit = 5.0;
    protected double sonicBallDistanceLimit = 5.0;
    protected double back2BasePower = -1* chargingPower;

    protected double leftArmFastAutoMovePower = 0.40;
    protected double numberBallsShoot = 2.0;
    protected double numberTimePressBeacon= 1.0;

    protected long lastTimeStamp = 0;

    protected long startWaitingTime = 1000;

    // jam detection
    protected JammingDetection  jammingDetection = null;

    protected boolean whiteLineFound = false;

    protected boolean pickUpBalls = true;

    // to do: add wall tracker

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        super.init();

        // line tracker
        hardwareLineTracker = new HardwareLineTracker();
        hardwareLineTracker.init(hardwareMap, 4);
        groundBrightness = Math.max(minLineBrightness,hardwareLineTracker.getBaseLineBrightness()*2.5);

        // beacon presser
        initBeaconPresser();

        // wall tracker
        initWallTracker();

        jammingDetection = new JammingDetection (1000L);

        state = 0;

        // load configuration

    }

    public void initBeaconPresser() {
        beaconPresser = new BeaconPresser(gyroTracker, leftBeaconArm);
        beaconPresser.setReporter(telemetry);
    }

    public void initWallTracker() {
        wallTracker = new WallTracker(wallTrackerHW,
                robot.motorLeftWheel,
                robot.motorRightWheel, 5);
        wallTracker.init();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        super.init_loop();
        beaconPresser.calibrate_loop();
        wallTracker.readDistance();
        adjustConfigurationViaGamePad();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        super.start();
        particleShooter.start(0);
        particleShooter.armPower = leftArmAutoMovePower;
        particleShooter.armStartPosition = leftArmMovePosition;
        particleShooter.handFirePower = 0.55;
        particleShooter.reload();
        beaconPresser.beaconArm.commitCalibration();
        beaconPresser.start(0);
        VortexUtils.moveMotorByEncoder(robot.motorLeftArm, leftArmMovePosition, leftArmAutoMovePower);
        lastTimeStamp = System.currentTimeMillis();
        whiteLineFound= false;
        state = 0;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("State:", "%02d", state);
        switch (state) {
            case 0:
                particleShooter.reload();
                // go straight
                gyroTracker.breakDistance = 800;
                state = gyroTracker.goStraight(0, cruisingTurnGain, searchingPower,
                        start2FireDistance, state, state + 1);

                // move arm
                particleShooter.moveArmToFirePosition();

                if (System.currentTimeMillis() - lastTimeStamp > 200) {
                    // move and raise arm at same time
                    state = gyroTracker.goStraight(0, cruisingTurnGain, cruisingPower,
                            start2FireDistance, state, state + 1);
                    particleShooter.reload();
                    particleShooter.relaxHand();
                } else {
                    // slow start to avoid turning
                    state = gyroTracker.goStraight(0, cruisingTurnGain, searchingPower,
                            start2FireDistance, state, state + 1);
                }

                // almost touch the ball
                //                wallTracker.readDistance();
                //                if (wallTracker.getHistoryDistanceAverage() < sonicBallDistanceLimit) {
                //                    state = 1;
                //                }

                if (state == 1) {
                    // prepare to shoot
                    robot.motorLeftWheel.setPower(0.0);
                    robot.motorRightWheel.setPower(0.0);
                    particleShooter.start(0);
                    particleShooter.armPower = leftArmAutoMovePower;
                    particleShooter.armStartPosition = leftArmFiringSafeZone - 700;
                }
                break;
            case 1:
                // shoot particles
                state = particleShooter.loop(state, state + 1);
                break;
            case 2:
                // turn 45 degree
                particleShooter.cock();
                state = gyroTracker.turn(fire2TurnDegree, inPlaceTurnGain,
                        turningPower, state, state + 1);

                if (state == 3) {
                    // activate jamming detection
                    jammingDetection.reset();
                }
                break;
            case 3:
                wallTracker.readDistance();

                // go straight until hit the wall
                gyroTracker.breakDistance = 1600;
                state = gyroTracker.goStraight(fire2TurnDegree, cruisingTurnGain,
                        cruisingPower, fire2WallDistance, state, state + 2); // need to +2 to skip jam backup

                double sonicDistance = wallTracker.getHistoryDistanceAverage();
                telemetry.addData("Wall Distance: ", "%02f", sonicDistance);
                int travelDistance = Math.min(robot.motorLeftWheel.getCurrentPosition(),
                        robot.motorRightWheel.getCurrentPosition());
                telemetry.addData("Travel Distance: ", travelDistance);

                // wall distance detection
                if ((Math.abs(travelDistance - gyroTracker.getWheelLandmark()) > fire2WallDistance * 0.6
                        && sonicDistance <= sonicWallDistanceLimit)) {
                    stopWheels();
                    gyroTracker.setWheelLandmark();
                    state = 5;
                }

                // jamming detection
                if (jammingDetection.isJammed(travelDistance)) {
                    gyroTracker.minTurnPower = 0.01;
                    gyroTracker.breakDistance = 100;
                    stopWheels();
                    gyroTracker.setWheelLandmark(); // important. otherwise it use last landmark
                    state = 4;
                }
                break;
            case 4:
                // if jammed, back up a little bit
                state = gyroTracker.goStraight(fire2TurnDegree, cruisingTurnGain,
                        -1.0 * searchingPower, jammingBackupDistance, state, state + 1);
                break;
            case 5:

                if (hardwareLineTracker.onWhiteLine(groundBrightness, 2)) {
                    whiteLineFound = true;
                }

                // turn -90 degree back
                state = gyroTracker.turn(fire2TurnDegree + wall2TurnDegree - 5,
                        inPlaceTurnGain, turningPower, state, state + 1);

                if (state == 6) {
                    // reset min turning power to avoid jerky movements
                    gyroTracker.minTurnPower = 0.01;
                    VortexUtils.moveMotorByEncoder(robot.motorLeftArm,
                            leftArmFiringSafeZone, leftArmAutoMovePower);
                    if (whiteLineFound) {
                        state = 7; // skip find white line step
                        stopWheels();
                        gyroTracker.setWheelLandmark();
                        beaconPresser.start(0);
                    }

                    if (pickUpBalls) {
                        robot.servoLeftScooper.setPower(leftScooperGo);
                        robot.servoRightScooper.setPower(rightScooperGo);
                    }
                }
                break;
            case 6:

                // check the ods for white line signal
                if (hardwareLineTracker.onWhiteLine(groundBrightness, 2)) {
                    state = 7;
                    stopWheels();
                    gyroTracker.setWheelLandmark();
                    beaconPresser.start(0);
                }

                // go straight until hit first white line
                gyroTracker.breakDistance = 200;
                state = gyroTracker.goStraight(fire2TurnDegree + wall2TurnDegree,
                        cruisingTurnGain, searchingPower, wall2BeaconDistance, state, state + 1);

                break;
            case 7:

                // touch beacon
                state = beaconPresser.loop(state, state + 1);
                if (state == 8) {
                    gyroTracker.setWheelLandmark();
                    lastTimeStamp = System.currentTimeMillis();
                }
                break;
            case 8:
                // go straight until hit the second white line
                gyroTracker.breakDistance = 200;
                int chargeDistance = gyroTracker.getWheelLandmarkOdometer();

                // check the ods for white line signal
                if (chargeDistance > 1000
                        && hardwareLineTracker.onWhiteLine(groundBrightness, 2)) {
                    state = 9;
                    stopWheels();
                    gyroTracker.setWheelLandmark();
                    beaconPresser.start(0);
                }

                if (chargeDistance > 2500 || System.currentTimeMillis() - lastTimeStamp > 1000) {
                    state = gyroTracker.goStraight(fire2TurnDegree + wall2TurnDegree,
                            cruisingTurnGain, searchingPower, beacon2BeaconDistance, state, state + 1);
                } else {
                    state = gyroTracker.goStraight(fire2TurnDegree + wall2TurnDegree,
                            cruisingTurnGain, cruisingPower, beacon2BeaconDistance, state, state + 1);
                }

                break;
            case 9:
                // touch beacon
                state = beaconPresser.loop(state, state + 1);

                if (state == 10) {
                    gyroTracker.setWheelLandmark();
                }
                break;
            case 10:
                // turn 45 degree
                state = gyroTracker.turn(fire2TurnDegree + wall2TurnDegree + beacon2ParkTurnDegree,
                        inPlaceTurnGain, parkTurningPower, state, state + 1);
                if (state == 11) {
                    gyroTracker.setWheelLandmark();
                    lastTimeStamp = System.currentTimeMillis();
                }
                break;
            case 11:
                if (pickUpBalls == true) {
                    if (robot.armStopMin.isPressed()) {
                        leftArmMinLimitSwitchOnCount++;
                    } else {
                        leftArmMinLimitSwitchOnCount = 0;
                    }
                    if (leftArmMinLimitSwitchOnCount > leftArmLimitSwitchCountThreshold) {
                        particleShooter.relaxArm();
                    } else {
                        VortexUtils.moveMotorByEncoder(robot.motorLeftArm,
                                leftArmHomeParkingPostion, leftArmAutoMovePower);
                    }
                }
                state = gyroTracker.goStraight(fire2TurnDegree + wall2TurnDegree + beacon2ParkTurnDegree,
                        cruisingTurnGain, searchingPower * 0.5, beacon2PickBallDistance, state, state + 1);
                if (state == 12) {
                    gyroTracker.setWheelLandmark();
                    lastTimeStamp = System.currentTimeMillis();
                }
                break;
            case 12:
                // backup straight to central parking
                gyroTracker.breakDistance = 0;
                robot.servoLeftScooper.setPower(leftScooperStop);
                robot.servoRightScooper.setPower(rightScooperStop);
                state = gyroTracker.goStraight(fire2TurnDegree + wall2TurnDegree + beacon2ParkTurnDegree,
                        cruisingTurnGain, back2BasePower, beacon2ParkingDistance, state, state + 1);

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

    protected void adjustConfigurationViaGamePad () {
        // adjust the number of balls to shoot
        if (gamepad1.b) {
            numberBallsShoot += 0.01;
        } else if (gamepad1.a) {
            numberBallsShoot -= 0.01;
            if (numberBallsShoot < 0.0) {
                numberBallsShoot = 0;
            }
        }
        particleShooter.autoShootCountLimit = (int) numberBallsShoot;
        telemetry.addData("Num of balls to shoot (a-/b+)    :", particleShooter.autoShootCountLimit);

        // adjust autonomous turning power
        if (gamepad1.dpad_up) {
            gyroTracker.minTurnPower += 0.0001;
        } else if (gamepad1.dpad_down) {
            gyroTracker.minTurnPower -= 0.0001;
            if (gyroTracker.minTurnPower < 0.0 ) {
                gyroTracker.minTurnPower = 0.0;
            }
        }
        telemetry.addData("Min turn power (pad up+/down-)   :", gyroTracker.minTurnPower);

        if (gamepad1.dpad_left) {
            gyroTracker.maxTurnPower += 0.0001;
        } else if (gamepad1.dpad_right) {
            gyroTracker.maxTurnPower -= 0.0001;
            if (gyroTracker.maxTurnPower < gyroTracker.minTurnPower) {
                gyroTracker.maxTurnPower = gyroTracker.minTurnPower + 0.01;
            }
        }
        telemetry.addData("Max turn power (pad left+/right-):", gyroTracker.maxTurnPower);

        // adjust waiting time
        if (gamepad1.left_bumper) {
            startWaitingTime += 10;
        } else if (gamepad1.right_bumper) {
            startWaitingTime -=10;
            if (startWaitingTime < 0 ) {
                startWaitingTime = 0;
            }
        }
        telemetry.addData("Waiting ms (bumper left+/right-) :", startWaitingTime);

        // enable/disable ball picking
        if (gamepad1.x) {
            pickUpBalls = true;
        } else if (gamepad1.y) {
            pickUpBalls = false;
        }
        telemetry.addData("Enable ball picking (x+/y-)    :", pickUpBalls);

        // number of time beacon to press
        if (gamepad2.a) {
            numberTimePressBeacon += 0.01;
        } else if (gamepad2.b) {
            numberTimePressBeacon -= 0.01;
            if (numberTimePressBeacon < 0.0) {
                numberTimePressBeacon = 0.0;
            }
        }
        beaconPresser.pressButtonTimesLimit = (int) numberTimePressBeacon;
        telemetry.addData("Press beacon times (a2+/b2-)       :", beaconPresser.pressButtonTimesLimit);

    }
}
