package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class BeaconPresser extends RobotExecutor {


    long lastTimeStamp = 0;

    DcMotor leftWheel = null;
    DcMotor rightWheel = null;

    public BeaconPresser(DcMotor left, DcMotor right){
        leftWheel = left;
        rightWheel = right;
    }

    @Override
    public void start(int startState) {

        state = startState;
        lastTimeStamp = System.currentTimeMillis();
    }



    @Override
    public int loop (int startState, int endState) {
        switch (state) {
            case 0:
                leftWheel.setPower(0.0);
                rightWheel.setPower(0.0);
                // move to the closer side of the beacon
                if (System.currentTimeMillis() - lastTimeStamp > 2000){ // wait 2 seconds
                    state = 1;
                }
                break;
            case 1:
                // detect beacon color
                state = 2;
                break;
            case 2:
                // touch beacon button
                state = 3;
                break;
            case 3:
                // move to the further side of the beacon
                state = 4;
                break;
            case 4:
                // detect beacon color
                state = 5;
                break;
            case 5:
                // touch beacon button
                state = 6;
                break;
            default:
                return endState;
        }
        return startState;
    }
}
