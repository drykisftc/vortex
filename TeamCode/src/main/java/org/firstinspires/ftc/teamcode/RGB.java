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

    double getDistance (RGB rgb) {
        return Math.sqrt((rgb.r-r)*(rgb.r-r)+(rgb.g-g)*(rgb.g-g)+(rgb.b-b)*(rgb.b-b));
    }
}
