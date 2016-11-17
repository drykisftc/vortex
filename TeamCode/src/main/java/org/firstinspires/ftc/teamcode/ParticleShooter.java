package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class ParticleShooter extends RobotExecutor {

    // arm
    protected int armStartPosition =0;
    protected int armFiringPosition1 = 4495;
    protected int armFiringPosition2 = 4495;
    protected double armPower = 0.45;
    protected int armFiringSafeZone = 3500;
    protected int leftArmPositionTolerance = 1;

    // hand
    int fireState =0;
    protected int handHomePosition =0;
    protected int handFirePosition =0;
    protected int handFirePositionOffset = 445; // 20: 1 motor is 560. 16:1 is 445
    protected int handFireOvershotOffset = 20; // 20:1 motor is 350. 16:1 is 230
    protected int handFireEncoderMissOffset = 0; // to compensate steps missed by encoders
    protected double handHoldPower = 0.05;
    protected double handBeakPower = 0.1;
    protected double handCalibrationPower = -0.05;
    protected double handFirePower = 1.0;
    protected int fireCount =0;
    protected long lastFireTimeStamp = 0;
    protected long minFireInterval = 2000;
    protected long minReloadInterval = 1500;
    protected boolean handReloaded = true;
    protected int leftHandFirePositionTolerance = 1;

    // cock servo
    protected double cockLoadPosition = 0.85;
    protected double cockFirePosition = 0.3;

    // devices
    protected DcMotor motorArm;
    protected DcMotor motorHand;
    protected Servo   servoCock;

    long lastTimeStamp = 0;

    public ParticleShooter(DcMotor arm,
                           DcMotor hand,
                           Servo servo){
        motorArm = arm;
        motorHand = hand;
        servoCock = servo;
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
        armStartPosition = motorArm.getCurrentPosition();
        servoCock.setPosition(cockLoadPosition);
        lastTimeStamp = System.currentTimeMillis();
    }

    public void reset () {
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
                if (System.currentTimeMillis() - lastTimeStamp < 500 ) {
                    // slow move first
                    VortexUtils.moveMotorByEncoder(motorArm, armFiringPosition1, armPower*0.5);
                } else {
                    VortexUtils.moveMotorByEncoder(motorArm, armFiringPosition1, armPower);
                }
                if (hasReachedPosition(armFiringPosition1)) {
                    state = 1;
                }
                if (reporter != null) {
                    reporter.addData("Particle shooter ", "move arm to 1st firing position");
                }
                break;
            case 1:
                // shoot the first particle
                if (isReady()) {
                    shoot_loop(true); // will set fire state to be not zero
                    if (isReady()) { // if ready again go next state
                        state = 2;
                    }
                    if (reporter != null) {
                        reporter.addData("Particle shooter ", "Fox 1");
                    }
                }
                break;
            case 2:
                // adjust arm position
                VortexUtils.moveMotorByEncoder(motorArm, armFiringPosition2, armPower);
                if (hasReachedPosition(armFiringPosition2)) {
                    state = 3;
                }
                if (reporter != null) {
                    reporter.addData("Particle shooter ", "move arm to 2nd firing position");
                }
                break;
            case 3:
                // shoot the second particle
                if (isReady()) {
                    shoot_loop(true);
                    if (isReady()) {
                        state = 4;
                    }
                    if (reporter != null) {
                        reporter.addData("Particle shooter ", "Fox 2");
                    }
                }
                break;
            case 4:
                // move arm back 
                VortexUtils.moveMotorByEncoder(motorArm, armStartPosition, armPower);
                if (reporter != null) {
                    reporter.addData("Particle shooter ", "move arm to back");
                }

                state = 5;
                break;
            default:
                return endState;
        }
        return startState;
    }

    public void shoot_loop(boolean triggerOn) {

        long currentT = System.currentTimeMillis();
        long timeSinceLastFiring = System.currentTimeMillis() - lastFireTimeStamp;

        if (!triggerOn) {
            fireState = 5;
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
                        lastFireTimeStamp = currentT;
                        handFirePosition
                                = currentHandP + handFirePositionOffset - handFireEncoderMissOffset;
                        handReloaded = false;
                        fireState = 1;
                        reporter.addData("Particle shooter", "Fox %d cocking......", fireCount);
                    } else {
                        reporter.addData("Particle shooter", "In danger zone, skip!!!");
                        fireState = 5; // skip to 5
                    }
                }
                break;
            case 1:
                if (servoCock.getPosition() == cockFirePosition) {
                    motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorHand.setPower(handFirePower);
                    reporter.addData("Particle shooter", "Fox %d fired!", fireCount);
                    fireState = 2;
                } else {
                    servoCock.setPosition(cockFirePosition);
                    reporter.addData("Particle shooter", "Fox %d cocking...", fireCount);
                }
                break;
            case 2:
                reporter.addData("Particle shooter", "Fox %d launching......", fireCount);
                if (currentHandP > handFirePosition + handFireOvershotOffset
                        || timeSinceLastFiring > minReloadInterval) {
                    motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorHand.setPower(0);
                    reporter.addData("Particle shooter", "Fox %d out!", fireCount);
                    fireState = 3;
                }
                break;
            case 3:
                VortexUtils.moveMotorByEncoder(motorHand, handFirePosition, handBeakPower);
                servoCock.setPosition(cockLoadPosition);
                fireState = 4;
                break;
            case 4:
                reporter.addData("Particle shooter", "Fox %d Reload......", fireCount);
                if (Math.abs(currentHandP - handFirePosition) <= leftHandFirePositionTolerance
                        || timeSinceLastFiring > minFireInterval) {
                    fireState = 5;
                }
            case 5:
            default:
                if (timeSinceLastFiring > minFireInterval ) {
                    VortexUtils.moveMotorByEncoder(motorHand, handFirePosition, handHoldPower);
                    servoCock.setPosition(cockLoadPosition);
                    fireState = 0;
                    handReloaded = true;
                    reporter.addData("Particle shooter", "Idle %d vs %d", currentHandP, handFirePosition);
                } else {
                    reporter.addData("Particle shooter", "Fox %d Reload......", fireCount);
                }
                break;
        }
    }

    protected boolean isReady () {
        return fireState == 0;
    }

    protected boolean hasReachedPosition ( int targetPos) {
        if ( Math.abs(motorArm.getCurrentPosition() - targetPos) < leftArmPositionTolerance) {
            return true;
        }
        return false;
    }

    public void calibrateHandByBall () {
        motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorHand.setPower(handCalibrationPower);
        handFirePosition = motorHand.getCurrentPosition();

    }

    public void relax () {
        motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorHand.setPower(0.0);
    }

    public boolean isHammerHomed() {
        return Math.abs(motorArm.getCurrentPosition() - handFirePosition) < handFirePositionOffset /8;
    }

}
