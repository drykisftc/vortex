package org.firstinspires.ftc.teamcode;

/**
 * Created by hfu on 10/25/16.
 */

public class Tracker extends RobotExecutor {

    // change this value for different robot to compensate the friction
    double minTurnPower = 0.02;
    double maxTurnPower = 0.5;
    double skewPowerGain = 1.0/100; // 180 for track wheels
    double skewTolerance = 0;

    /**
     *
     * @param skew
     * @return delta power = (+/-)minTurnPower + gain * skew
     */
    protected  double computeTurnPower (double skew ) {
        double deltaPower = 0.0;
        if (Math.abs(skew) > skewTolerance) {
            deltaPower = skewPowerGain * skew;
            // always apply minimum force to compensate the friction
            if (deltaPower > 0.0) {
                deltaPower += minTurnPower;
            } else if (deltaPower < 0.0  ) {
                deltaPower -= minTurnPower;
            }
        }
        return Math.min(maxTurnPower,deltaPower);
    }

}
