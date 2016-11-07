package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ParticleShooter extends RobotExecutor {

    // arm
    protected int armStartPosition =0;
    protected int armFiringPosition1 = 4000;
    protected int armFiringPosition2 = 4000;
    protected double armPower = 0.45;
    protected int leftArmFiringSafeZone = 2000;
    protected int leftArmPosotionTolerance = 5;

    // hand
    int fireState =0;
    protected int handHomePosition =0;
    protected int handFirePositionOffset = 445; // 20: 1 motor is 560. 16:1 is 445
    protected int handFireOvershotOffset = 230; // 20:1 motor is 350. 16:1 is 230
    protected double handHoldPower = 0.05;
    protected double handFirePower = 1.0;
    protected int fireCount =0;
    protected long lastFireTimeStamp = 0;
    protected long minFireInterval = 1800;
    protected long minReloadInterval = 1200;
    protected boolean handReloaded = true;
    
    protected int leftHandFirePositionTolerance = 0;
    
    protected DcMotor motorArm;
    protected DcMotor motorHand;

    long      latTimeStamp = 0;

    public ParticleShooter(DcMotor arm,
                           DcMotor hand){
        motorArm = arm;
        motorHand = hand;
    }
    
    @Override
    public void start(int s) {
        state = s;
        // get current arm position
        motorHand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        handHomePosition = motorHand.getCurrentPosition();
        motorHand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorHand.setTargetPosition(handHomePosition);
        motorHand.setPower(handHoldPower);
        fireCount = 1; // must preload it to be 1
        armStartPosition = motorArm.getCurrentPosition();
        latTimeStamp = System.currentTimeMillis();
    }

    public int loop (int startState, int endState) {
        switch (state) {
            case 0:
                // move arm to firing position
                if (System.currentTimeMillis() - latTimeStamp < 500 ) {
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
                fireLoop();
                if (fireState >= 2) {
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
                fireLoop();
                if (fireState >=2 ) {
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
    
    public void fireLoop () {

        long currentT = System.currentTimeMillis();
        long timeSinceLastFiring = System.currentTimeMillis() - lastFireTimeStamp;
        int currentP = motorArm.getCurrentPosition();
        int fireP = handHomePosition + fireCount * handFirePositionOffset;

        switch (fireState) {
            case 0:
                // fire
                if (handReloaded
                        && timeSinceLastFiring > minFireInterval) {
                    if (currentP > leftArmFiringSafeZone) {
                        // fire
                        fireCount++;
                        lastFireTimeStamp = currentT;
                        VortexUtils.moveMotorByEncoder(motorHand,
                                fireP + handFireOvershotOffset + handFirePositionOffset,
                                handFirePower);
                        fireState = 1; // go to reload state
                    }
                }
                break;
            case 1:
                // reload
                if (timeSinceLastFiring > minReloadInterval) {
                    // reload
                    VortexUtils.moveMotorByEncoder(motorHand,
                            fireP, handFirePower * 0.5);
                    fireState = 2;
                }
            case 2:
                if (Math.abs(currentP - fireP) <= leftHandFirePositionTolerance) {
                    // assume that reload is done
                    handReloaded = true;
                    fireState = 3;
                }
                break;
            case 3:
            default:
                // settle to ready position
                VortexUtils.moveMotorByEncoder(motorHand,
                        handHomePosition + fireCount * handFirePositionOffset,
                        handHoldPower);
                handReloaded = true;
                break;
        }
    }

    protected boolean hasReachedPosition ( int targetPos) {
        if ( Math.abs(motorArm.getCurrentPosition() - targetPos) < leftArmPosotionTolerance ) {
            return true;
        }
        return false;
    }
    
    public void shoot (boolean triggerOn) {
        if (triggerOn) {
            // fire only when the arm is in fire-safe zone
            int currentArmP = motorArm.getCurrentPosition();
            if (currentArmP > leftArmFiringSafeZone) {
                long currentT = System.currentTimeMillis();
                int fireP = getNextFirePosition();
                if (handReloaded && currentT - lastFireTimeStamp < minFireInterval) {
                    // fire
                    fireCount++;
                    lastFireTimeStamp = currentT;

                    // encoder mode is slow. just use full power.
                    motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorHand.setPower(handFirePower);
                    handReloaded = false;

                } else if (motorHand.getCurrentPosition() > fireP) {
                    // reload
                    VortexUtils.moveMotorByEncoder(motorHand,
                            fireP, handFirePower * 0.5);
                } else {
                    // reload is done
                    handReloaded = true;
                }
            }
        } else {
            VortexUtils.moveMotorByEncoder(motorHand,
                    getNextFirePosition(),
                    handHoldPower);
            handReloaded = true;
        }
    }

    protected int getNextFirePosition () {
        int nexPos = handHomePosition + fireCount * handFirePositionOffset;
        reporter.addData("Particle shooter", "%2d", nexPos);
        return nexPos;
    }
}
