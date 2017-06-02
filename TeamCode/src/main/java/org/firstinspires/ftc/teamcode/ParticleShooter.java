package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

class ParticleShooter extends RobotExecutor {

    // arm
    int armStartPosition = 0;
    int armFiringPosition = 500;
    int armFiringSafeZone = 200;
    double armPower = 0.45;
    private int armFiringPositionAdjust = 0;
    private int leftArmPositionTolerance = 10;

    // hand
    double handFirePower = 0.55;
    private int fireState = 0;
    private int handHomePosition = 0;
    private int handFirePosition = 0;
    private int handFirePositionOffset = 445; // 20: 1 motor is 560. 16:1 is 445
    private int handFireOvershotOffset = -80; //
    private int handFireBreakingOffset = 30;
    private int handFireBreakingOffset2 = 10;
    private int handFireEncoderMissOffset = 0; // to compensate steps missed by encoders
    private int handCalibrationOffset = 35;
    private double handHoldPower = 0.05;
    private double handBeakPower = 0.25;
    private double handCalibrationPower = -0.05;
    private double handPressPower = -0.15;
    private int fireCount = 0;
    private long lastFireTimeStamp = 0;
    private long minFireInterval = 1800;
    private long minReloadInterval = 1200;
    private long pressBallInterval = 300;
    private boolean handReloaded = true;
    private boolean autoShootEnded = true;
    private int leftHandFirePositionTolerance = 5;

    // cock servo
    private double cockLoadPosition = 0.50;
    private double cockFirePosition = 0.01;

    // devices
    private DcMotor motorArm;
    private DcMotor motorHand;
    private Servo servoCock;
    private TouchSensor limitSwitch;

    private long lastTimeStamp = 0;

    private int limitSwitchCount = 0;
    private int limitSwitchCountThreshold = 4;

    private int autoShootCount = 0;
    public int autoShootCountLimit = 2;

    // jam detection
    private JammingDetection  jammingDetection = null;

    ParticleShooter(DcMotor arm,
                    DcMotor hand,
                    Servo servo,
                    TouchSensor armLimitSensor) {
        motorArm = arm;
        motorHand = hand;
        servoCock = servo;
        limitSwitch = armLimitSensor;
        jammingDetection = new JammingDetection(1000L);
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
        reload();
        lastTimeStamp = System.currentTimeMillis();
    }

    @Override
    public void start(int s) {
        state = s;
        fireState = 0;
        reload();
        autoShootEnded = false;
        autoShootCount = 0;
        lastTimeStamp = System.currentTimeMillis();
    }

    void reset() {
        handReloaded = true;
        reload();
        motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorHand.setPower(0);
        fireState = 0;
        autoShootCount = 0;
    }

    public int loop(int startState, int endState) {

        int currentArmP = motorArm.getCurrentPosition();
        int currentHandP = motorHand.getCurrentPosition();
        reporter.addData("Particle shooter arm  pos", currentArmP);
        reporter.addData("Particle shooter hand pos", currentHandP);

        switch (state) {
            case 0:
                // move arm to firing position
                reload();
                VortexUtils.moveMotorByEncoder(motorArm, armFiringPosition, armPower);
                if (hasReachedPosition(armFiringPosition)) {
                    state = 1;
                }
                if (isArmJammed(motorArm.getCurrentPosition())
                        || isLimitSwitchOn()) {
                    state = 1;
                    relaxArm();
                }
                if (reporter != null) {
                    reporter.addData("Particle shooter ", "move arm to 1st firing position");
                }
                break;
            case 1:
                if (System.currentTimeMillis() - lastTimeStamp < 300) {
                    reload();
                } else {
                    state = 2;
                    cock();
                    relaxArm();
                    lastTimeStamp = System.currentTimeMillis();
                }
                break;
            case 2:
                if (System.currentTimeMillis() - lastTimeStamp < 600) {
                    cock();
                } else {
                    state = 3;
                    relaxArm();
                    lastTimeStamp = System.currentTimeMillis();
                }
                break;
            case 3:
                if (System.currentTimeMillis() - lastTimeStamp < 200) {
                    cock();
                    calibrateHandByBall();
                    relaxArm();
                } else {
                    state = 4;
                    autoShootEnded = false;
                    lastTimeStamp = System.currentTimeMillis();
                }
                break;
            case 4:
                // shoot the first particle
                shoot_loop(true, handFirePower);
                if (isReadyToShoot()) { // if ready again go next state
                    reload();
                    autoShootCount++;
                    lastTimeStamp = System.currentTimeMillis();
                    state = 5;
                }
                if (reporter != null) {
                    reporter.addData("Particle shooter ", "Fox %02d", autoShootCount);
                }
                break;
            case 5:
                reload();
                relaxArm();
                if (autoShootCount < autoShootCountLimit) {
                    state = 1;
                } else {
                    state = 6;
                }
                lastTimeStamp = System.currentTimeMillis();
                break;
            case 6:
                VortexUtils.moveMotorByEncoder(motorHand,
                        handFirePosition,
                        handHoldPower);
                VortexUtils.moveMotorByEncoder(motorArm, armStartPosition, armPower);
                state = 7;
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
                        cock();
                        pressBall();
                        lastFireTimeStamp = currentT;
                        handFirePosition = currentHandP + handFirePositionOffset - handFireEncoderMissOffset;
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
                    motorHand.setPower(Range.clip(power, 0.0, 1.0));
                    reporter.addData("Particle shooter", "Fox %d fired!", fireCount);
                    fireState = 3;
                } else {
                    cock();
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
                reload();
                // move short to prevent hammer jamming the ball, break point 1
                VortexUtils.moveMotorByEncoder(motorHand, handFirePosition+handFireBreakingOffset, handBeakPower);
                fireState = 5;
                break;
            case 5:
                reporter.addData("Particle shooter", "Fox %d Reload......", fireCount);
                if ((currentHandP - handFirePosition- handFireBreakingOffset) < 0) {
                    // break point 2
                    VortexUtils.moveMotorByEncoder(motorHand, handFirePosition+handFireBreakingOffset2, handHoldPower);
                    handReloaded = true;
                    fireState = 6;
                } else if (timeSinceLastFiring > minReloadInterval) {
                    fireState = 6;
                }
            case 6:
            default:
                if (timeSinceLastFiring > minFireInterval
                        || Math.abs(currentHandP-handFirePosition-handFireBreakingOffset2) < leftHandFirePositionTolerance ) {
                    VortexUtils.moveMotorByEncoder(motorHand, handFirePosition, handHoldPower);
                    reload();
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

    private boolean hasReachedPosition(int targetPos) {
        return Math.abs(motorArm.getCurrentPosition() - targetPos) < leftArmPositionTolerance;
    }

    boolean isLimitSwitchOn() {
        if (limitSwitch.isPressed()) {
            limitSwitchCount++;
        } else {
            limitSwitchCount = 0;
        }
        return limitSwitchCount >= limitSwitchCountThreshold;
    }

    void calibrateHandByBall() {
        motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorHand.setPower(handCalibrationPower);
        handFirePosition = motorHand.getCurrentPosition()
                + handCalibrationOffset; // add a little free room to allow next ball fall-in
    }

    void pressBall() {
        motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorHand.setPower(handPressPower);
        cock();
    }

    void releaseBall() {
        motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorHand.setPower(-1.0 * handCalibrationPower);
        reload();
    }

    void relaxHand() {
        motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorHand.setPower(0.0);
    }

    void relaxArm() {
        // Notice: don't set motor mode back to to USING_encoder. It will make encoder losing steps
        motorArm.setPower(0.0);
    }

    void slowDownArm() {
        // Notice: don't set motor mode back to to USING_encoder. It will make encoder losing steps
        motorArm.setPower(armPower/10.0);
    }

    void reload() {
        servoCock.setPosition(cockLoadPosition);
    }
    
    void cock() {
        servoCock.setPosition(cockFirePosition);
    }


    boolean isHammerHomed() {
        return Math.abs(motorHand.getCurrentPosition() - handFirePosition) < leftArmPositionTolerance;
    }

    void resetArmJammed() {
        jammingDetection.reset();
    }

    boolean isArmJammed(int position) {
        return jammingDetection.isJammed(position);
    }

    public void moveArmToFirePosition () {
        if (isArmJammed(motorArm.getCurrentPosition())
                || isLimitSwitchOn()) {
            relaxArm();
        } else {
            VortexUtils.moveMotorByEncoder(motorArm,
                    armFiringPosition, armPower);
        }
    }

}
