package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareBase
{

    HardwareMap hwMap           =  null;

    /* Constructor */
    public HardwareBase(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
    }

    public void reset () {

    }
}
