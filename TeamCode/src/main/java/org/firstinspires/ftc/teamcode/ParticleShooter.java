package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.DecimalFormat;

public class ParticleShooter extends Excecutor {


    public ParticleShooter(){

    }

    public int  loop (int startState, int endState) {
        switch (state) {
            case 0:
                // move to the closer side of the beacon
                break;
            case 1:
                // detect beacon color
                break;
            case 2:
                // touch beacon button
                break;
            case 3:
                // move to the further side of the beacon
                break;
            case 4:
                // detect beacon color
                break;
            case 5:
                // touch beacon button
                break;
            default:
                return endState;
        }
        return startState;
    }

}
