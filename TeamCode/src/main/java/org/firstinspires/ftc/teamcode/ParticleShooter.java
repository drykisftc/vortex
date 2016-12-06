package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

class ParticleShooter extends RobotExecutor {

    // arm
    private int armStartPosition =0;
    int armFiringPosition = 500;
    private int armFiringPositionAdjust = 0;
    private double armPower = 0.45;
    int armFiringSafeZone = 500;
    private int leftArmPositionTolerance = 50;

    // hand
    private int fireState =0;
    private int handHomePosition =0;
    private int handFirePosition =0;
    private int handFirePositionOffset = 445; // 20: 1 motor is 560. 16:1 is 445
    private int handFireOvershotOffset = -65; //
    private int handFireEncoderMissOffset = 0; // to compensate steps missed by encoders
    private int handCalibrationOffset = 15;
    private double handHoldPower = 0.05;
    private double handBeakPower = 0.15;
    private double handCalibrationPower = -0.05;
    private double handFirePower = 1.0;
    double handFirePowerAttenuate = 0.6;
    private int fireCount =0;
    private long lastFireTimeStamp = 0;
    private long minFireInterval = 1800;
    private long minReloadInterval = 1200;
    private long pressBallInterval = 300;
    private boolean handReloaded = true;
    private boolean autoShootEnded = true;
    private int leftHandFirePositionTolerance = 1;

    // cock servo
    private double cockLoadPosition = 0.85;
    private double cockFirePosition = 0.3;

    // devices
    private DcMotor motorArm;
    private DcMotor motorHand;
    private Servo   servoCock;
    private TouchSensor limitSwitch;

    private long lastTimeStamp = 0;

    private int limitSwitchCount = 0;
    private int limitSwitchCountThreshold = 10;

    // jam detection
    private int armJamPosition =0;
    private long armJammedTime =0;
    private long armJammedTimeLimit = 1000;

    ParticleShooter(DcMotor arm,
                    DcMotor hand,
                    Servo servo,
                    TouchSensor armLimitSensor){
        motorArm = arm;
        motorHand = hand;
        servoCock = servo;
        limitSwitch = armLimitSensor;
    }

    @Override
     public void init() {
        // get current arm position
        motorHand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        handHomePosition = motorHand.getCurrentPosition();
        motorHand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorHand.setTargetPosition(handHomePosition);
        motorHand.setPower(handHoldPower);
        fireCount = 0; // must preload it to be 0
        armStartPosition = motorArm.getCurrentPosition();
        servoCock.setPosition(cockLoadPosition);
        lastTimeStamp = System.currentTimeMillis();
    }
    @Override
     public void start(int s) {
        state = s;
        fireState = 0;
        armStartPosition = motorArm.getCurrentPosition();
        servoCock.setPosition(cockLoadPosition);
        autoShootEnded = false;
        lastTimeStamp = System.currentTimeMillis();
    }

    void reset () {
        handReloaded = true;
        servoCock.setPosition(cockLoadPosition);
        motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorHand.setPower(0);
        fireState = 0;
    }

     public int loop (int startState, int endState) {
        switch (state) {
            case 0:
                // move arm to firing position
                servoCock.setPosition(cockLoadPosition);
                int targetPos = armFiringPosition-armFiringPositionAdjust;
                if (System.currentTimeMillis() - lastTimeStamp < 500) {
                    VortexUtils.moveMotorByEncoder(motorArm, targetPos, armPower * 0.5);
                } else {
                    VortexUtils.moveMotorByEncoder(motorArm, targetPos, armPower);
                }
                if (hasReachedPosition(targetPos)) {
                    state = 1;
                }
                if (isArmJammed(motorArm.getCurrentPosition())
                        || isLimitSwitchOn()) {
                    state = 1;
                    motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorArm.setPower(0.0);
                }
                if (reporter != null) {
                    reporter.addData("Particle shooter ", "move arm to 1st firing position");
                }
                break;
            case 1:
                if ( System.currentTimeMillis() - lastTimeStamp < 500){
                    servoCock.setPosition(cockLoadPosition);
                } else {
                    state = 2;
                    lastTimeStamp = System.currentTimeMillis();
                }
                break;
            case 2:
                if ( System.currentTimeMillis() - lastTimeStamp < 800){
                    servoCock.setPosition(cockFirePosition);
                    calibrateHandByBall();
                } else {
                    state = 3;
                    servoCock.setPosition(cockLoadPosition);
                    lastTimeStamp = System.currentTimeMillis();
                }
                break;
            case 3:
                if ( System.currentTimeMillis() - lastTimeStamp < 1000){
                    servoCock.setPosition(cockLoadPosition);
                } else {
                    state = 4;
                    lastTimeStamp = System.currentTimeMillis();
                }
                break;
            case 4:
                // wait half second to recharge
                if ( System.currentTimeMillis() - lastTimeStamp > 500){
                    state = 5;
                    motorArm.setPower(0.0);
                    servoCock.setPosition(cockFirePosition);
                    autoShootEnded = false;
                    lastTimeStamp = System.currentTimeMillis();
                }
                break;
            case 5:
                // shoot the first particle
                shoot_loop(true,Range.clip(handFirePower*handFirePowerAttenuate,0.01,1.0));
                if (isReadyToShoot()) { // if ready again go next state
                    servoCock.setPosition(cockLoadPosition);
                    state = 6;
                }
                if (reporter != null) {
                    reporter.addData("Particle shooter ", "Fox 1");
                }
                break;
            case 6:
                // adjust arm position
                VortexUtils.moveMotorByEncoder(motorArm, armFiringPosition, armPower);
                if (hasReachedPosition(armFiringPosition)) {
                    state = 7;
                    servoCock.setPosition(cockLoadPosition);
                    lastTimeStamp = System.currentTimeMillis();
                }
                if (reporter != null) {
                    reporter.addData("Particle shooter ", "move arm to 2nd firing position");
                }
                break;
            case 7:
                if ( System.currentTimeMillis() - lastTimeStamp < 1000){
                    servoCock.setPosition(cockLoadPosition);
                    autoShootEnded = false;
                } else {
                    state = 8;
                    lastTimeStamp = System.currentTimeMillis();
                }
                break;
            case 8:
                // shoot the second particle
                shoot_loop(true, Range.clip(handFirePower*handFirePowerAttenuate,0.01,1.0));
                if (isReadyToShoot()) {
                    state = 9;
                }
                if (reporter != null) {
                    reporter.addData("Particle shooter ", "Fox 2");
                }
                break;
            case 9:
                // move arm back
                VortexUtils.moveMotorByEncoder(motorArm, armStartPosition, armPower);
                if (reporter != null) {
                    reporter.addData("Particle shooter ", "move arm to back");
                }
                state = 10;
                break;
            default:
                return endState;
        }
        return startState;
    }

    void shoot_loop(boolean triggerOn, double power) {

        long currentT = System.currentTimeMillis();
        long timeSinceLastFiring = System.currentTimeMillis() - lastFireTimeStamp;

        if (!triggerOn) {
            fireState = 6;
        } else {
            reporter.addData("Particle shooter", "Hot!!");
        }

        int currentArmP = motorArm.getCurrentPosition();
        int currentHandP = motorHand.getCurrentPosition();

        switch (fireState) {
            case 0:
                reporter.addData("Particle shooter", "Fox %d firing......", fireCount);
                // fire
                if (handReloaded
                        && timeSinceLastFiring > minFireInterval) {
                    if (currentArmP > armFiringSafeZone) {
                        // fire
                        fireCount++;
                        servoCock.setPosition(cockFirePosition);
                        pressBall();
                        lastFireTimeStamp = currentT;
                        handFirePosition
                                = currentHandP + handFirePositionOffset - handFireEncoderMissOffset;
                        handReloaded = false;
                        fireState = 1;
                        reporter.addData("Particle shooter", "Fox %d cocking......", fireCount);
                    } else {
                        reporter.addData("Particle shooter", "In danger zone, skip!!!");
                        fireState = 6; // skip to 6
                    }
                }
                break;
            case 1:
                if (timeSinceLastFiring < pressBallInterval) {
                    motorHand.setPower(handCalibrationPower);
                } else {
                    fireState = 2;
                }
                break;
            case 2:
                if (servoCock.getPosition() == cockFirePosition) {
                    motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorHand.setPower(power);
                    reporter.addData("Particle shooter", "Fox %d fired!", fireCount);
                    fireState = 3;
                } else {
                    servoCock.setPosition(cockFirePosition);
                    reporter.addData("Particle shooter", "Fox %d cocking...", fireCount);
                }
                break;
            case 3:
                reporter.addData("Particle shooter", "Fox %d launching......", fireCount);
                if (currentHandP > handFirePosition + handFireOvershotOffset
                        || timeSinceLastFiring > minReloadInterval) {
                    servoCock.setPosition(cockLoadPosition);
                    //motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorHand.setPower(0);
                    reporter.addData("Particle shooter", "Fox %d out!", fireCount);
                    fireState = 4;
                }
                break;
            case 4:
                servoCock.setPosition(cockLoadPosition);
                VortexUtils.moveMotorByEncoder(motorHand, handFirePosition, handBeakPower);
                fireState = 5;
                break;
            case 5:
                reporter.addData("Particle shooter", "Fox %d Reload......", fireCount);
                if (Math.abs(currentHandP - handFirePosition) <= leftHandFirePositionTolerance) {
                    handReloaded = true;
                    fireState = 6;
                } else if ( timeSinceLastFiring > minReloadInterval) {
                    fireState = 6;
                }
            case 6:
            default:
                if (timeSinceLastFiring > minFireInterval
                        || Math.abs(currentHandP - handFirePosition) <= leftHandFirePositionTolerance) {
                    VortexUtils.moveMotorByEncoder(motorHand, handFirePosition, handHoldPower);
                    servoCock.setPosition(cockLoadPosition);
                    fireState = 0;
                    handReloaded = true;
                    autoShootEnded = true;
                    reporter.addData("Particle shooter", "Idle %d vs %d", currentHandP, handFirePosition);
                } else {
                    reporter.addData("Particle shooter", "Fox %d Reload......", fireCount);
                }
                break;
        }
    }

    private boolean isReadyToShoot() {
        return fireState == 0 && autoShootEnded == true;
    }

    private boolean hasReachedPosition ( int targetPos) {
        return Math.abs( motorArm.getCurrentPosition() - targetPos) < leftArmPositionTolerance;
    }

    private  boolean isLimitSwitchOn () {
        if (limitSwitch.isPressed()) {
            limitSwitchCount ++;
        } else {
            limitSwitchCount = 0;
        }
        return limitSwitchCount >= limitSwitchCountThreshold;
    }

    void calibrateHandByBall () {
        pressBall();
        handFirePosition = motorHand.getCurrentPosition()
                +handCalibrationOffset; // add a little free room to allow next ball fall-in
    }

     void pressBall () {
        motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorHand.setPower(handCalibrationPower);
        servoCock.setPosition(cockFirePosition);
    }

     void releaseBall () {
        motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorHand.setPower(-1.0*handCalibrationPower);
        handFirePosition = motorHand.getCurrentPosition();
        servoCock.setPosition(cockLoadPosition);
    }

     void relax () {
        motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorHand.setPower(0.0);
    }

     boolean isHammerHomed() {
        return Math.abs(motorArm.getCurrentPosition() - handFirePosition) < leftArmPositionTolerance;
    }

    protected void resetArmJammed () {
        armJammedTime = System.currentTimeMillis();
    }

    protected boolean isArmJammed (int position) {
        if ( armJamPosition != position) {
            armJammedTime = System.currentTimeMillis();
            armJamPosition = position;
            return false;
        } else if (System.currentTimeMillis()- armJammedTime > armJammedTimeLimit)  {
            return true;
        } else {
            return false;
        }
    }

}
