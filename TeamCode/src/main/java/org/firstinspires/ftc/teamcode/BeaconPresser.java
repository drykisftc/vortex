package org.firstinspires.ftc.teamcode;

public class BeaconPresser extends RobotExecutor {

    GyroTracker gyroTracker = null;
    HardwareBeaconArm beaconArm = null;

    protected long lastTimeStamp = 0;

    // navigation info
    protected int lineToBeaconDistance = 400; //509
    protected int beaconPressDistance = 2500;
    protected int button1ToButton2Distance = 486;
    double cruisingPower = 0.3;
    double cruisingTurnGain = 0.002;
    int distanceThreshold = 2;
    char teamColor = 'b';

    // bookkeeping
    int landMarkAngle = 0;
    boolean bBeaconPressed = false;
    int teamColorCount = 0;
    int teamColorCountThreshold = 6;

    protected long pressTimeLimit = 2000; // 3 seconds
    protected long travelTimeLimit = 10000; // 10 seconds

    protected double upperArmSlowStepSize = 0.01;
    protected double lowerArmSlowStepSize = 0.05;
    protected double upperArmFastStepSize = 0.01;
    protected double lowerArmFastStepSize = 0.05;

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
        beaconArm.commitCalibration();
        distanceThreshold = beaconArm.colorSensorAmbient + 2;
        upperArmFastStepSize = beaconArm.upperArmStepSize;
        upperArmSlowStepSize = beaconArm.upperArmStepSize /3;
        lowerArmFastStepSize = beaconArm.lowerArmStepSize;
        lowerArmSlowStepSize = beaconArm.lowerArmStepSize /3;
    }

    public void calibrate () {
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
                    beaconArm.upperArmStepSize= upperArmSlowStepSize;
                    beaconArm.lowerArmStepSize= lowerArmSlowStepSize;
                    lastTimeStamp = System.currentTimeMillis();
                }
                break;
            case 1:
                // extend arm
                if ( beaconArm.extendUntilNearLoop(distanceThreshold)
                        || System.currentTimeMillis() - lastTimeStamp > travelTimeLimit) {
                    state = 2;
                }
                break;
            case 2:
                // move slowly until it gets the team color
                state = gyroTracker.goStraight (landMarkAngle, cruisingTurnGain, cruisingPower,
                        beaconPressDistance, 2,3);

                // hover beacon arm over beacon
                beaconArm.extendNear(distanceThreshold);

                if (isColor(teamColor)) {
                    state = 3;
                }

                if (state == 3 ) {
                    gyroTracker.setWheelLandmark();
                    gyroTracker.stopWheels();
                    beaconArm.upperArmStepSize= upperArmFastStepSize;
                    beaconArm.lowerArmStepSize= lowerArmFastStepSize;
                    beaconArm.retract();
                    lastTimeStamp = System.currentTimeMillis();
                }
                break;
            case 3:
                // touch beacon button
                if (beaconArm.extendUntilTouch()
                        || System.currentTimeMillis() - lastTimeStamp > pressTimeLimit){
                    state = 4;
                    bBeaconPressed = true;
                    beaconArm.retract();
                }
                break;
            case 4:
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

    public int loop2 (int startState, int endState) {
        if (reporter != null) {
            reporter.addData("BeaconPresser State:", "%02d", state);
        }
        switch (state) {
            case 0:
                // move to first beacon button
                state = gyroTracker.goStraight (landMarkAngle, cruisingTurnGain, cruisingPower,
                        lineToBeaconDistance, 0,1);
                if (state == 1) {
                    lastTimeStamp = System.currentTimeMillis();
                }
                break;
            case 1:
                // extend arm
                if ( beaconArm.extendUntilNearLoop(distanceThreshold)
                        || System.currentTimeMillis() - lastTimeStamp > travelTimeLimit) {
                    state = 2;
                }
                break;
            case 2:
                // detect beacon color
                if (beaconArm.getColorBlueOrRed() == teamColor) {
                    state = 3;
                    lastTimeStamp = System.currentTimeMillis();
                } else {
                    beaconArm.retract();
                    state = 4;
                }
                break;
            case 3:
                // touch beacon button
                if (beaconArm.extendUntilTouch()
                        || System.currentTimeMillis() - lastTimeStamp > travelTimeLimit){
                    state = 4;
                    bBeaconPressed = true;
                    beaconArm.retract();
                }
                break;
            case 4:
                // move to the further side of the beacon
                state = gyroTracker.goStraight (landMarkAngle, cruisingTurnGain, cruisingPower,
                        button1ToButton2Distance, 0,1);
                if (state == 5) {
                    lastTimeStamp = System.currentTimeMillis();
                }
                break;
            case 5:
                if (bBeaconPressed) {
                    state = 8;
                } else if ( beaconArm.extendUntilNearLoop(distanceThreshold)
                        || System.currentTimeMillis() - lastTimeStamp > travelTimeLimit) {
                    state = 6;
                }
                break;
            case 6:
                if (beaconArm.getColorBlueOrRed() == teamColor) {
                    state = 7;
                    lastTimeStamp = System.currentTimeMillis();
                } else {
                    state = 8;
                }
                break;
            case 7:
                if (beaconArm.extendUntilTouch()
                        || System.currentTimeMillis() - lastTimeStamp > travelTimeLimit) {
                    // touch button
                    state = 8;
                    bBeaconPressed = true;
                }
                break;
            case 8:
            default: {
                beaconArm.retract();
                return endState;
            }
        }
        return startState;
    }
}
