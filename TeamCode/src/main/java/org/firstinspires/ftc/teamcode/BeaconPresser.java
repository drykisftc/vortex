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
    double searchingPower = 0.15;
    double cruisingTurnGain = 0.002;
    int distanceThreshold = 2;
    char teamColor = 'b';

    // bookkeeping
    int landMarkAngle = 0;
    boolean bBeaconPressed = false;
    int teamColorCount = 0;
    int teamColorCountThreshold = 6;

    double slowSpeedGain = 0.1;
    double fastSpeedGain = 1.0;

    protected long pressTimeLimit = 2000; // 3 seconds
    protected long travelTimeLimit = 10000; // 10 seconds


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
                if (beaconArm.extendUntilTouch(fastSpeedGain)
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
}
