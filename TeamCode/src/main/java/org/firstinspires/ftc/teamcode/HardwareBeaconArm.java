package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
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
    double upperArmMax = 0.99;
    double upperArmMin = 0.45;
    protected double upperArmCurrentPosition = 0.0;

    Servo lowerArm = null;
    String lowerArmName = "lowerArm";
    double lowerArmHomePosition = 1.0;
    double lowerArmStepSize = 0.01;
    double lowerArmMax = 0.99;
    double lowerArmMin = 0.01;
    protected double lowerArmCurrentPosistion = 0;

    ColorSensor colorSensor = null;
    String colorSensorName = "beaconArmColor";
    int colorSensorAddress = 0x1e;  // color sensor default is 0x3c. in 7 bit, it is  0x3c/2 = 0x1e
    int colorSensorAmbient = 0;
    protected int calibrationCount = 0;
    final protected int calibrationCountLimit = 10000;
    int colorSensorForegroundThreshold = 0;

    TouchSensor touchSensor = null;
    String touchSensorName = "beaconArmTouch";
    protected int touchCounts = 0;
    protected int touchCountLimit = 4;

    protected int nearCounts = 0;
    protected int nearCountsLimit = 3;

    protected int numbOfSteps =0;

    protected int state = 0;

    protected Random random = new Random(System.currentTimeMillis());

    protected RGB ambientRGB = new RGB(0,0,0);

    HardwareBeaconArm ( String upArmName, String lowArmName,
                        String colorName, String touchName) {
        upperArmName = upArmName;
        lowerArmName = lowArmName;
        colorSensorName = colorName;
        touchSensorName = touchName;
    }

    HardwareBeaconArm ( String upArmName, String lowArmName,
                        String colorName, int colorSensorAddr,
                        String touchName) {
        upperArmName = upArmName;
        lowerArmName = lowArmName;
        colorSensorName = colorName;
        colorSensorAddress = colorSensorAddr;
        touchSensorName = touchName;
    }

    public void init(HardwareMap ahwMap) {

        super.init(ahwMap);

        upperArm =  hwMap.servo.get(upperArmName);
        lowerArm = hwMap.servo.get(lowerArmName);
        colorSensor = hwMap.colorSensor.get(colorSensorName);
        colorSensor.setI2cAddress(I2cAddr.create7bit(colorSensorAddress));
        touchSensor = hwMap.touchSensor.get(touchSensorName);

        colorSensor.enableLed(false);

        // reset calibration data
        calibrationCount = 0;
        ambientRGB.fillZero();

    }

    public void start (double upperHome, double lowerHome,
                       double upStepSize, double lowStepSize) {
        upperArmHomePosition = upperHome;
        upperArmMin = upperHome;
        upperArmMax = upperHome + upStepSize/Math.abs(upStepSize)* 0.45;
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
    public boolean extendUntilNearLoop ( int intensityThreshold, double speedGain) {

        if (getColorIntensity() >= intensityThreshold) {
            nearCounts++;
        } else {
            nearCounts = 0;
        }

        if (nearCounts < nearCountsLimit) {
            extend(speedGain);
            return false;
        }
        return true;
    }

    /**
     * @return false if not touch yet
     */
    public boolean extendUntilTouch (double speedGain) {

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
            extend(speedGain);
        } else {
            // shake it
            //shake();
        }

        return bT;
    }

    public void hoverNear(int target, double speedGain) {

        if (getColorIntensity() > target) {
            upperArm.setPosition(Range.clip(upperArm.getPosition()-numbOfSteps * upperArmStepSize*speedGain,
                    upperArmMin, upperArmMax));
            lowerArm.setPosition(Range.clip(lowerArm.getPosition()-numbOfSteps * lowerArmStepSize*speedGain,
                    lowerArmMin, lowerArmMax));
        } else {
            upperArm.setPosition(Range.clip(upperArm.getPosition()+numbOfSteps * upperArmStepSize*speedGain,
                    upperArmMin, upperArmMax));
            lowerArm.setPosition(Range.clip(lowerArm.getPosition()+numbOfSteps * lowerArmStepSize*speedGain,
                    lowerArmMin, lowerArmMax));
        }
    }

    public void extend ( double speedGain ) {
        numbOfSteps ++;
        upperArm.setPosition(Range.clip(upperArmHomePosition+numbOfSteps*upperArmStepSize*speedGain,
                upperArmMin, upperArmMax));
        lowerArm.setPosition(Range.clip(lowerArmHomePosition+numbOfSteps*lowerArmStepSize*speedGain,
                lowerArmMin, lowerArmMax));
    }

    public void shake ( double speedGain ) {
        numbOfSteps = numbOfSteps + random.nextInt(9) - 4;
        upperArm.setPosition(Range.clip(upperArmHomePosition+numbOfSteps*upperArmStepSize*speedGain,
                upperArmMin, upperArmMax));
        lowerArm.setPosition(Range.clip(lowerArmHomePosition+numbOfSteps*lowerArmStepSize*speedGain,
                lowerArmMin, lowerArmMax));
    }

    public void retract () {
        numbOfSteps =0;
        upperArm.setPosition(upperArmHomePosition);
        lowerArm.setPosition(lowerArmHomePosition);
    }

    /**
     *
     * @return less than 0 if red, bigger than 0 if blue
     */
    public int isBlueOrRed () {
        return colorSensor.blue() - colorSensor.red();
    }

    /**
     * Don't care about green color
     * @return team color in red or blue
     */
    public char getColorBlueOrRed () {
        int d = isBlueOrRed();
        if ( d > 1) {
            return 'b';
        } else if (d < -1) {
            return 'r';
        } else {
            return 'u';
        }
    }

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
        if (calibrationCount < calibrationCountLimit) {
            calibrationCount++;
            ambientRGB.r += colorSensor.red();
            ambientRGB.g += colorSensor.green();
            ambientRGB.b += colorSensor.blue();
        }
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
        colorSensorForegroundThreshold = colorSensorAmbient+2;
    }

    public void pressButton_loop(double speedGain) {
        switch (state) {
            case 0:
                resetCounters();
                retract();
                break;
            case 1:
                if (extendUntilNearLoop(colorSensorForegroundThreshold, speedGain)) ;
                break;
            case 2:
                extendUntilTouch(speedGain);
                break;
            case 3:
                hoverNear(colorSensorForegroundThreshold, speedGain);
                break;
            default:
                break;
        }
    }
}
