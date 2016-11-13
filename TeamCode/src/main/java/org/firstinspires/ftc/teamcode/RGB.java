package org.firstinspires.ftc.teamcode;

public class RGB {
    int r=0;
    int g=0;
    int b=0;

    public RGB(int rr, int gg, int bb){
        r = rr;
        g = gg;
        b = bb;
    }

    int getIntensity(){
        return r+g+b;
    }
}