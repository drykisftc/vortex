package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class BeaconPresser extends RobotExecutor {

    GyroTracker gyroTracker = null;
    HardwareBeaconArm beaconArm = null;

    protected long lastTimeStamp = 0;

    // navigation info
    protected int lineToBeaconDistance = 250;
    protected int button1ToButton2Distance = 50;
    double cruisingPower = 0.4;
    double cruisingTurnGain = 0.002;
    int distanceThreshold = 2;
    char teamColor = 'b';

    // bookkeeping
    int landMarkAngle = 0;
    boolean bBeaconPressed = false;

    protected long timeLimit = 3000; // 3 seconds

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
                    lastTimeStamp = System.currentTimeMillis();
                }
                break;
            case 1:
                // extend arm
                if ( beaconArm.extendUntilNearLoop(distanceThreshold)
                        || System.currentTimeMillis() - lastTimeStamp > timeLimit ) {
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
                        || System.currentTimeMillis() - lastTimeStamp > timeLimit ){
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
                        || System.currentTimeMillis() - lastTimeStamp > timeLimit) {
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
                        || System.currentTimeMillis() - lastTimeStamp > timeLimit) {
                    // touch button
                    state = 8;
                    bBeaconPressed = true;
                }
                break;
            case 8:
            default:
                beaconArm.retract();
                return endState;
        }
        return startState;
    }
}
