package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class HardwareBeaconArm extends HardwareBase {

    Servo upperArm = null;
    String upperArmName = "upperArm";
    double upperArmHomePosition = 0.0;
    double upperArmStepSize = 0.01;

    Servo lowerArm = null;
    String lowerArmName = "lowerArm";
    double lowerArmHomePosition = 1.0;
    double lowerArmStepSize = 0.01;

    ColorSensor colorSensor = null;
    String colorSensorName = "beaconArmColor";

    TouchSensor touchSensor = null;
    String touchSensorName = "beaconArmTouch";
    int touchCounts = 0;
    int touchCountLimit = 10;

    HardwareBeaconArm ( String upArmName, String lowArmName,
                        String colorName, String touchName) {
        upperArmName = upArmName;
        lowerArmName = lowArmName;
        colorSensorName = colorName;
        touchSensorName = touchName;
    }

    public void init(HardwareMap ahwMap) {

        super.init(ahwMap);

        upperArm =  hwMap.servo.get(upperArmName);
        lowerArm = hwMap.servo.get(lowerArmName);
        colorSensor = hwMap.colorSensor.get(colorSensorName);
        touchSensor = hwMap.touchSensor.get(touchSensorName);
    }

    public void start (double upperHome, double lowerHome,
                       double upStepSize, double lowStepSize) {
        upperArmHomePosition = upperHome;
        lowerArmHomePosition = lowerHome;
        upperArmStepSize = upStepSize;
        lowerArmStepSize = lowStepSize;
    }

    public void reset () {
        retract();
        touchCounts = 0;
    }

    /**
     *
     * @param intensityTheshold
     * @return false if not near yet
     */
    public boolean extendUntilNearLoop ( int intensityTheshold) {
        if (getColorIntensity() < intensityTheshold) {
            extend();
            return false;
        }
        return true;
    }

    /**
     * @return false if not touch yet
     */
    public boolean extendUntilTouch () {

        boolean bT = false ;

        if (touchSensor.isPressed()) {
            touchCounts ++ ;
        }

        if (touchCounts >= touchCountLimit) {
            bT = true;
            touchCounts = touchCountLimit;
        }

        if (!bT) {
            extend();
        }

        return bT;
    }

    public void extend ( ) {
        upperArm.setPosition(upperArm.getPosition()+upperArmStepSize);
        lowerArm.setPosition(lowerArm.getPosition()+lowerArmStepSize);
    }

    public void retract () {
        upperArm.setPosition(upperArmHomePosition);
        lowerArm.setPosition(lowerArmHomePosition);
    }


    public char getColor () {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        int m = Math.max(Math.max(r,g),b);
        if ( m == r ) {
            return 'r';
        } else if ( m == g ) {
            return 'g';
        }
        return 'b';
    }

    public double getColorDistance (RGB rgb) {
        RGB c = new RGB (colorSensor.red(),colorSensor.green(),colorSensor.blue());
        return c.getDistance(rgb);
    }

    public int getColorIntensity() {
        return colorSensor.red() + colorSensor.green() + colorSensor.blue();
    }

}
