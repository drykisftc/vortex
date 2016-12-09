package org.firstinspires.ftc.teamcode;

public class JammingDetection {

    private int jamPosition = 0;
    private long jammedTime = 0;
    private long jammedTimeLimit = 1000;

    public JammingDetection (Long timeLimit) {
        jammedTimeLimit = timeLimit;
    }

    protected void reset () {
        jammedTime = System.currentTimeMillis();
    }

    protected boolean isJammed (int position) {
        if (jamPosition != position) {
            jammedTime = System.currentTimeMillis();
            jamPosition = position;
            return false;
        } else if (System.currentTimeMillis() - jammedTime > jammedTimeLimit) {
            return true;
        } else {
            return false;
        }
    }
}
