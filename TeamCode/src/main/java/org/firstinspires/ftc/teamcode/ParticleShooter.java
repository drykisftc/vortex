package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ParticleShooter extends RobotExecutor {

    // arm
    protected int armStartPosition =0;
    protected int armFiringPosition1 = 4500;
    protected int armFiringPosition2 = 4500;
    protected double armPower = 0.45;
    protected int leftArmFiringSafeZone = 3500;
    protected int leftArmPositionTolerance = 5;

    // hand
    int fireState =0;
    protected int handHomePosition =0;
    protected int handFirePosition =0;
    protected int handFirePositionOffset = 445; // 20: 1 motor is 560. 16:1 is 445
    protected int handFireOvershotOffset = 20; // 20:1 motor is 350. 16:1 is 230
    protected double handHoldPower = 0.02;
    protected double handBeakPower = 0.01;
    protected double handCalibrationPower = -0.01;
    protected double handFirePower = 1.0;
    protected int fireCount =0;
    protected long lastFireTimeStamp = 0;
    protected long minFireInterval = 1800;
    protected long minReloadInterval = 1200;
    protected boolean handReloaded = true;
    
    protected int leftHandFirePositionTolerance = 1;
    
    protected DcMotor motorArm;
    protected DcMotor motorHand;

    long lastTimeStamp = 0;

    public ParticleShooter(DcMotor arm,
                           DcMotor hand){
        motorArm = arm;
        motorHand = hand;
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
        lastTimeStamp = System.currentTimeMillis();
    }
    @Override
    public void start(int s) {
        state = s;
        armStartPosition = motorArm.getCurrentPosition();
        lastTimeStamp = System.currentTimeMillis();
    }

    public void reset () {
        handReloaded = true;
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
                    fireState = 0;
                    state = 1;
                }
                if (reporter != null) {
                    reporter.addData("Particle shooter ", "move arm to 1st firing position");
                }
                break;
            case 1:
                // shoot the first particle
                shoot_loop(true);
                if (fireState >= 3) {
                    state = 2;
                }
                if (reporter != null) {
                    reporter.addData("Particle shooter ", "Fox 1");
                }
                break;
            case 2:
                // adjust arm position
                VortexUtils.moveMotorByEncoder(motorArm, armFiringPosition2, armPower);
                if (hasReachedPosition(armFiringPosition2)) {
                    fireState = 0;
                    state = 3;
                }
                if (reporter != null) {
                    reporter.addData("Particle shooter ", "move arm to 2nd firing position");
                }
                break;
            case 3:
                // shoot the second particle
                shoot_loop(true);
                if (fireState >=3 ) {
                    state = 4;
                }
                if (reporter != null) {
                    reporter.addData("Particle shooter ", "Fox 2");
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
            fireState = 3;
        }

        int currentArmP = motorArm.getCurrentPosition();
        int currentHandP = motorHand.getCurrentPosition();

        switch (fireState) {
            case 0:
                // fire
                if (handReloaded
                        && timeSinceLastFiring > minFireInterval) {
                    if (currentArmP > leftArmFiringSafeZone) {
                        // fire
                        fireCount++;
                        lastFireTimeStamp = currentT;
                        motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        motorHand.setPower(handFirePower);
                        handFirePosition = currentHandP + handFirePositionOffset;
                        handReloaded = false;
                        fireState = 1;
                    } else {
                        fireState = 3; // skip to 3
                    }
                }
                break;
            case 1:
                if (currentHandP > handFirePosition + handFireOvershotOffset) {
                    VortexUtils.moveMotorByEncoder(motorHand,
                            handFirePosition, handBeakPower);
                    fireState = 2; // go to reload state
                }
                break;
            case 2:
                if (Math.abs(currentHandP - handFirePosition) <= leftHandFirePositionTolerance
                        || timeSinceLastFiring > minFireInterval) {
                    fireState = 3;
                    handReloaded = true;
                }
            case 3:
            default:
                VortexUtils.moveMotorByEncoder(motorHand,
                        handFirePosition, handHoldPower);
                fireState = 0;
                break;
        }
    }

    protected boolean hasReachedPosition ( int targetPos) {
        if ( Math.abs(motorArm.getCurrentPosition() - targetPos) < leftArmPositionTolerance) {
            return true;
        }
        return false;
    }
    
    public void shoot (boolean triggerOn) {

        long currentT = System.currentTimeMillis();
        if (triggerOn) {
            reporter.addData("Particle shooter", "Fire!");
            // fire only when the arm is in fire-safe zone
            int currentArmP = motorArm.getCurrentPosition();
            if (currentArmP > leftArmFiringSafeZone) {
                int currentHand = motorHand.getCurrentPosition();
                if (handReloaded && currentT - lastFireTimeStamp > minFireInterval) {
                    // bookkeeping
                    fireCount++;
                    lastFireTimeStamp = currentT;

                    // fire
                    motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorHand.setPower(handFirePower);
                    handFirePosition = currentHand + handFirePositionOffset;
                    handReloaded = false;
                    reporter.addData("Particle shooter", "Fox %d", fireCount);

                } else if ( currentHand > handFirePosition + handFireOvershotOffset) {
                    // reload
                    VortexUtils.moveMotorByEncoder(motorHand,
                            handFirePosition, handBeakPower);
                    reporter.addData("Particle shooter", "Reload" );
                } else if (Math.abs(currentHand - handFirePosition) < leftHandFirePositionTolerance){
                    // reload is done
                    handReloaded = true;
                    reporter.addData("Particle shooter", "Ready");
                    motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorHand.setPower(0);
                }
            }
        } else if (currentT - lastFireTimeStamp > minFireInterval){
            VortexUtils.moveMotorByEncoder(motorHand,
                    handFirePosition,
                    handHoldPower);
            handReloaded = true;
        }
    }

    public void calibrateHandByBall() {
        motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorHand.setPower(handCalibrationPower);
    }

//    protected int getNextFirePosition () {
//        int nexPos = handHomePosition + fireCount * handFirePositionOffset;
//        reporter.addData("Particle shooter", "fire count %2d", fireCount);
//        reporter.addData("Particle shooter", "hand pos   %2d", nexPos);
//        return nexPos;
//    }
}
