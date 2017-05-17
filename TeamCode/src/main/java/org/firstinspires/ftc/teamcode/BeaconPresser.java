package org.firstinspires.ftc.teamcode;

import java.util.Random;

public class BeaconPresser extends RobotExecutor {

    GyroTracker gyroTracker = null;
    HardwareBeaconArm beaconArm = null;

    protected long lastTimeStamp = 0;

    // navigation info
    protected int lineToBeaconDistance = 390; //509
    protected int beaconPressDistance = 750;
    protected int button1ToButton2Distance = 486;
    double cruisingPower = 0.4;
    double searchingPower = 0.15;
    double cruisingTurnGain = 0.002;
    int distanceThreshold = 1;
    char teamColor = 'b';

    int pressButtonTimes = 0;
    int pressButtonTimesLimit = 2;
    int pressButtonTimesMaxLimit = 6;

    // bookkeeping
    int landMarkAngle = 0;
    boolean bBeaconPressed = false;
    int teamColorCount = 0;
    int teamColorCountThreshold = 2;

    double slowSpeedGain = 0.09;
    double fastSpeedGain = 1.0;

    protected long longPressTimeLimit = 1000; // 1.5 seconds
    protected long shotPressTimeLimit = 80; // 0.3 seconds
    protected long travelTimeLimit = 4000; // 4 seconds

    protected int waggleDegree = 0;
    protected double waggleGain = 0.01;
    protected int waggleDistance = 100;
    protected double wagglePower = searchingPower;

    Random random = new Random();

    public BeaconPresser(GyroTracker g,
                         HardwareBeaconArm arm){
        gyroTracker = g;
        beaconArm = arm;
    }

    @Override
    public void start(int startState) {
        super.start(startState);
        lastTimeStamp = System.currentTimeMillis();
        landMarkAngle = gyroTracker.gyro.getHeading();
        bBeaconPressed = false;
        pressButtonTimes = 0;
    }

    public void calibrate_loop () {
        beaconArm.calibrate_loop();
    }

    @Override
    public int loop (int startState, int endState) {
        if (reporter != null) {
            reporter.addData("BeaconPresser State:", "%02d", state);
        }
        switch (state) {
            case 0:
                // move to first beacon button
                state = gyroTracker.goStraight (landMarkAngle, cruisingTurnGain, cruisingPower,
                        lineToBeaconDistance, 0,1);
                if (state == 1) {
                    // make arm moving slow enough to see intensity change
                    lastTimeStamp = System.currentTimeMillis();
                }
                break;
            case 1:
                // extend arm
                if ( beaconArm.extendUntilNearLoop(distanceThreshold, slowSpeedGain)
                        || System.currentTimeMillis() - lastTimeStamp > travelTimeLimit) {
                    state = 2;
                }
                break;
            case 2:

                // hover beacon arm over beacon
                beaconArm.hoverNear(distanceThreshold,slowSpeedGain);

                // move slowly until it gets the team color, if not found skip to end state
                state = gyroTracker.goStraight (landMarkAngle, cruisingTurnGain, searchingPower,
                        beaconPressDistance, 2,6);

                // check team color
                if (isColor(teamColor)) {
                    state = 3;
                }

                if (state == 3 ) {
                    gyroTracker.setWheelLandmark();
                    gyroTracker.stopWheels();
                    beaconArm.retract();
                    lastTimeStamp = System.currentTimeMillis();
                }

                if (pressButtonTimes >= pressButtonTimesLimit) {
                    state = 7;
                }
                break;
            case 3:
                // touch beacon button
                // waggle wheels to touch beacon at different angles
                //gyroTracker.turn(landMarkAngle+waggleDegree*((pressButtonTimes+2)%3-1), waggleGain, 0.0,state, state);
                beaconArm.extend(fastSpeedGain);
                if (System.currentTimeMillis() - lastTimeStamp > longPressTimeLimit){
                    pressButtonTimes ++;
                    bBeaconPressed = true;
                    lastTimeStamp = System.currentTimeMillis();
                    if (pressButtonTimes >= pressButtonTimesLimit) {
                        state = 7;

                    } else {
                        state = 4;
                    }
                }
                break;
            case 4:
                // waggle wheels to touch beacon at different angles
                //gyroTracker.turn(landMarkAngle+waggleDegree*((pressButtonTimes+2)%3-1), waggleGain, 0.0,state, state);

                // touch beacon button
                beaconArm.retract();
                if (System.currentTimeMillis() - lastTimeStamp > shotPressTimeLimit){
                    state = 5;
                    pressButtonTimes ++;
                    bBeaconPressed = true;
                    gyroTracker.setWheelLandmark();
                    lastTimeStamp = System.currentTimeMillis();
                }
                break;
            case 5:
                gyroTracker.goStraight (landMarkAngle, cruisingTurnGain, wagglePower,
                        waggleDistance, state,state);
                beaconArm.extend(fastSpeedGain);
                if (System.currentTimeMillis() - lastTimeStamp > shotPressTimeLimit*3) {
                    if (pressButtonTimes >= pressButtonTimesLimit) {
                        state = 6;
                        gyroTracker.stopWheels();
                        lastTimeStamp = System.currentTimeMillis();
                    } else {
                        state = 4;
                        lastTimeStamp = System.currentTimeMillis();
                        waggleDegree *= -1.0; // flip the waggle angle
                        wagglePower *= -1.0;
                    }
                }
                break;
            case 6:
                if (pressButtonTimes >= pressButtonTimesMaxLimit
                        || isColorSatisfied(teamColor)) {
                    state = 7;
                }

                gyroTracker.goStraight (landMarkAngle, cruisingTurnGain, wagglePower,
                        waggleDistance, state,state);
                // randomly change direction
                if ( random.nextInt(2)== 0) {
                    wagglePower *=-1.0;
                }

                // hover beacon arm over beacon
                if (beaconArm.hoverNear(distanceThreshold,slowSpeedGain)>=0) {
                    // check beacon color again
                    if (!isColorSatisfied(teamColor)) {
                        state = 4;
                    } else {
                        state = 7;
                    }
                }
                break;
            default: {
                gyroTracker.stopWheels();
                gyroTracker.setWheelLandmark();
                lastTimeStamp = System.currentTimeMillis();
                beaconArm.retract();
                return endState;
            }
        }
        return startState;
    }

    private boolean isColor (char color) {
        if (beaconArm.getColorBlueOrRed() == color) {
            teamColorCount ++;
        } else {
            teamColorCount = 0;
        }
        return teamColorCount >= teamColorCountThreshold;
    }

    private boolean isColorSatisfied (char color) {
        char c = beaconArm.getColorBlueOrRed();
        if ( c == color || c == 'u') {
            teamColorCount ++;
        } else {
            teamColorCount = 0;
        }
        return teamColorCount >= teamColorCountThreshold;
    }

    void setNumOfTimesPressBeacon (int times ) {
        if (times > 0) {
            pressButtonTimesLimit = (int) times;
            pressButtonTimesMaxLimit = times*10;
        }
    }
}
