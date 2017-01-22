package org.firstinspires.ftc.teamcode;

/**
 * Created by andrewwang on 1/15/17.
 */

public class AccelerationOdometer extends RobotExecutor{
    public int tick = 100;
    private long lasttimestamp = 0;
    private int state = 0;
    public void loop() {
        switch(state){
            case 1:
                lasttimestamp = System.currentTimeMillis();
                //set time stamp
                state = 2;
                break;
            case 2:
                //wait until tick
                if (lasttimestamp <= System.currentTimeMillis() - tick){
                    state = 3;
                }
                break;
            default:
                state = 1;
                break;
        }
    }
}
