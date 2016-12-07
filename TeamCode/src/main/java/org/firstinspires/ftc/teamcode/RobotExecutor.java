package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.DecimalFormat;

public class RobotExecutor {

    Telemetry reporter = null;
    int state = 0;

    protected static DecimalFormat df3 = new DecimalFormat(".###");

    public RobotExecutor(){

    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init() {


    }

    /**
     *
     * @param startState, The initial state of wall follower
     */
    public void start(int startState) {

        state = startState;
    }

    public int loop (int startState, int endState) {
        // null operation
        return endState;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    public void stop() {

    }

    public void setReporter (Telemetry t){
        reporter = t;
    }

    public void report (String key, String value) {
        if (reporter != null) {
            reporter.addData(key, value);
        }
    }
}
