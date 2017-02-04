package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by hfu on 10/25/16.
 */

public class Tracker extends RobotExecutor {

    // change this value for different robot to compensate the friction
    double minTurnPower = 0.02;
    double maxTurnPower = 0.15;
    double skewPowerGain = 1.0/180; // 180 for track wheels

    double skewTolerance = 0;
    int settleCount =0;
    int settleCountLimit = 3;

    /**
     *
     * @param skew
     * @return delta power = (+/-)minTurnPower + gain * skew
     */
    protected  double computeTurnPower (double skew ) {
        double deltaPower = 0.0;

        if (Math.abs(skew) > skewTolerance) {
            settleCount = 0;
        } else {
            settleCount ++;
        }

        if (settleCount < settleCountLimit) {
            deltaPower = skewPowerGain * skew;
            // always apply minimum force to compensate the friction
            if (deltaPower > 0.0) {
                deltaPower += minTurnPower;
            } else if (deltaPower < 0.0  ) {
                deltaPower -= minTurnPower;
            }
        }
        return Range.clip(deltaPower, -1*maxTurnPower, maxTurnPower);
    }

    protected  double computeTurnPowerByRatio (double skew, double maxSkew, double power) {
        double deltaPower = 0.0;
        if (Math.abs(skew) > skewTolerance) {
            settleCount = 0;
        } else {
            settleCount ++;
        }
        if (settleCount < settleCountLimit) {
            deltaPower = power * skewPowerGain * skew/maxSkew;
            // always apply minimum force to compensate the friction
            if (deltaPower > 0.0) {
                deltaPower += minTurnPower;
            } else if (deltaPower < 0.0  ) {
                deltaPower -= minTurnPower;
            }
        }
        return Range.clip(deltaPower, -1*maxTurnPower, maxTurnPower);
    }

}
