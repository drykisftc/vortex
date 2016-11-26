package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
//import org.apache.commons.collections.buffer.CircularFifoBuffer;

import java.util.Random;

public class HardwareBeaconArm extends HardwareBase {

    Servo upperArm = null;
    String upperArmName = "upperArm";
    double upperArmHomePosition = 0.0;
    double upperArmStepSize = 0.01;
    double upperArmCurrentPosition = 0.0;

    Servo lowerArm = null;
    String lowerArmName = "lowerArm";
    double lowerArmHomePosition = 1.0;
    double lowerArmStepSize = 0.01;
    double lowerArmCurrentPosistion = 0;

    ColorSensor colorSensor = null;
    String colorSensorName = "beaconArmColor";
    int colorSensorAmbient = 0;
    int calibrationCount = 0;

    TouchSensor touchSensor = null;
    String touchSensorName = "beaconArmTouch";
    int touchCounts = 0;
    int touchCountLimit = 4;

    int nearCounts = 0;
    int nearCountsLimit = 3;

    int numbOfSteps =0;

    int state = 0;

    Random random = new Random(System.currentTimeMillis());

    RGB ambientRGB = new RGB(0,0,0);

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

        colorSensor.enableLed(false);

        // reset calibration data
        calibrationCount = 0;
        ambientRGB.fillZero();

    }

    public void start (double upperHome, double lowerHome,
                       double upStepSize, double lowStepSize) {
        upperArmHomePosition = upperHome;
        lowerArmHomePosition = lowerHome;
        upperArmStepSize = upStepSize;
        lowerArmStepSize = lowStepSize;
        colorSensor.enableLed(false);

        updatePosition();
    }

    public void reset () {
        retract();
        resetCounters();
        colorSensor.enableLed(false);
    }

    public void resetCounters() {
        touchCounts =0;
        nearCounts =0;
    }
    /**
     *
     * @param intensityThreshold
     * @return false if not near yet
     */
    public boolean extendUntilNearLoop ( int intensityThreshold) {

        if (getColorIntensity() > intensityThreshold) {
            nearCounts++;
        } else {
            nearCounts = 0;
        }

        if (nearCounts < nearCountsLimit) {
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
        } else {
            touchCounts =0;
        }

        if (touchCounts >= touchCountLimit) {
            bT = true;
            touchCounts = touchCountLimit;
        }

        if (!bT) {
            extend();
        } else {
            // shake it
            //shake();
        }

        return bT;
    }

    public void extend ( ) {
        numbOfSteps ++;
        upperArm.setPosition(Range.clip(upperArmHomePosition+numbOfSteps*upperArmStepSize, 0.45, 0.99));
        lowerArm.setPosition(Range.clip(lowerArmHomePosition+numbOfSteps*lowerArmStepSize, 0.01, 0.99));
    }

    public void shake ( ) {
        numbOfSteps = numbOfSteps + random.nextInt(9) - 4;
        upperArm.setPosition(Range.clip(upperArmHomePosition+numbOfSteps*upperArmStepSize, 0.45, 0.99));
        lowerArm.setPosition(Range.clip(lowerArmHomePosition+numbOfSteps*lowerArmStepSize, 0.01, 0.99));
    }

    public void retract () {
        numbOfSteps =0;
        upperArm.setPosition(upperArmHomePosition);
        lowerArm.setPosition(lowerArmHomePosition);
    }

    /**
     * Don't care about green color
     * @return team color in red or blue
     */
    public char getColor () {
        int r = colorSensor.red() - ambientRGB.r;
        //int g = colorSensor.green()- ambientRGB.g;
        int b = colorSensor.blue() - ambientRGB.b;

        //int m = Math.max(Math.max(r,g),b);
        int m = Math.max(r,b);
        if ( m == r ) {
            return 'r';
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

    public void updatePosition () {
        upperArmCurrentPosition  = upperArm.getPosition();
        lowerArmCurrentPosistion = lowerArm.getPosition();
    }

    public void calibrate_loop () {
        // compute ambient rgb
        calibrationCount ++;
        ambientRGB.r += colorSensor.red();
        ambientRGB.g += colorSensor.green();
        ambientRGB.b += colorSensor.blue();
    }

    public void commitCalibration () {
        // compute ambient intensity
        if (calibrationCount <=0) {
            ambientRGB.r = colorSensor.red();
            ambientRGB.g = colorSensor.green();
            ambientRGB.b = colorSensor.blue();
            colorSensorAmbient = ambientRGB.r+ambientRGB.g+ambientRGB.b;
        } else {
            ambientRGB.r /= calibrationCount;
            ambientRGB.g /= calibrationCount;
            ambientRGB.b /= calibrationCount;
            colorSensorAmbient = (ambientRGB.r + ambientRGB.g + ambientRGB.b)/calibrationCount;
        }
    }

    public void pressButton_loop(boolean bGoNext) {
        switch (state) {
            case 1:
                if (extendUntilNearLoop((int)(colorSensorAmbient*1.6))
                        && bGoNext) {
                    state = 2; // go to touch
                }
                break;
            case 2:
                if ( extendUntilTouch()
                        && bGoNext) {
                    state = 0; // retract
                }
                break;
            default:
                resetCounters();
                retract();
                break;
        }
    }
}
