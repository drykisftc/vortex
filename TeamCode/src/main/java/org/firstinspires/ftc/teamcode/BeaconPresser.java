package org.firstinspires.ftc.teamcode;

public class BeaconPresser extends RobotExecutor {

    GyroTracker gyroTracker = null;
    HardwareBeaconArm beaconArm = null;

    protected long lastTimeStamp = 0;

    // navigation info
    protected int lineToBeaconDistance = 400; //509
    protected int beaconPressDistance = 2500;
    protected int button1ToButton2Distance = 486;
    double cruisingPower = 0.5;
    double searchingPower = 0.2;
    double cruisingTurnGain = 0.002;
    int distanceThreshold = 1;
    char teamColor = 'b';

    int pressButtonTimes = 0;
    int pressButtonTimesLimit = 1;

    // bookkeeping
    int landMarkAngle = 0;
    boolean bBeaconPressed = false;
    int teamColorCount = 0;
    int teamColorCountThreshold = 3;

    double slowSpeedGain = 0.2;
    double fastSpeedGain = 3.0;

    protected long longPressTimeLimit = 1500; // 1.5 seconds
    protected long shotPressTimeLimit = 500; // 0.3 seconds
    protected long travelTimeLimit = 4000; // 4 seconds

    protected int waggleDegree = 2;
    protected double waggleGain = 0.01;

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
        distanceThreshold = beaconArm.colorSensorAmbient + 1;
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
                // move slowly until it gets the team color
                state = gyroTracker.goStraight (landMarkAngle, cruisingTurnGain, searchingPower,
                        beaconPressDistance, 2,3);

                // hover beacon arm over beacon
                beaconArm.hoverNear(distanceThreshold,slowSpeedGain);

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
                break;
            case 3:
                // touch beacon button
                // waggle wheels to touch beacon at different angles
                gyroTracker.skewTolerance = 0;
                gyroTracker.turn(landMarkAngle+waggleDegree*((pressButtonTimes+2)%3-1), waggleGain,
                        0.0,state, state);
                beaconArm.extend(fastSpeedGain);
                if (System.currentTimeMillis() - lastTimeStamp > longPressTimeLimit){
                    pressButtonTimes ++;
                    bBeaconPressed = true;
                    lastTimeStamp = System.currentTimeMillis();
                    beaconArm.retract();
                    state = 4;
                }
                break;
            case 4:
                // waggle wheels to touch beacon at different angles
                gyroTracker.skewTolerance = 0;
                gyroTracker.turn(landMarkAngle+waggleDegree*((pressButtonTimes+2)%3-1), waggleGain,
                        0.0,state, state);

                // touch beacon button
                beaconArm.extend(fastSpeedGain);
                if (System.currentTimeMillis() - lastTimeStamp > shotPressTimeLimit){
                    state = 5;
                    pressButtonTimes ++;
                    bBeaconPressed = true;
                    lastTimeStamp = System.currentTimeMillis();
                    beaconArm.retract();
                }
                break;
            case 5:
                if (System.currentTimeMillis() - lastTimeStamp < shotPressTimeLimit)  {
                    beaconArm.retract();
                } else if (pressButtonTimes >= pressButtonTimesLimit) {
                    state = 6;
                } else {
                    state = 4;
                    lastTimeStamp = System.currentTimeMillis();
                    waggleDegree *= -1.0; // flip the waggle angle
                }
                break;
            default: {
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
        return teamColorCount > teamColorCountThreshold;
    }
}
