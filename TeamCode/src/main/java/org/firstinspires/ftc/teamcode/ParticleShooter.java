package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ParticleShooter extends RobotExecutor {

    // arm
    protected int armStartPosition =0;
    protected int armFiringPosition1 = 4000;
    protected int armFiringPosition2 = 4000;
    protected double armPower = 0.45;
    protected int leftArmFiringSafeZone = 2000;
    protected int leftArmPositionTolerance = 5;

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
    
    protected int leftHandFirePositionTolerance = 1;
    
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
        fireCount = 0; // must preload it to be 0
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
                shoot(true);
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
                shoot(true);
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
    
    public void shoot(boolean triggerOn) {

        long currentT = System.currentTimeMillis();
        long timeSinceLastFiring = System.currentTimeMillis() - lastFireTimeStamp;
        int fireP = getNextFirePosition();

        if (!triggerOn) {
            fireState = 2;
            // settle to ready position
            VortexUtils.moveMotorByEncoder(motorHand,
                    fireP,
                    handHoldPower);
            if (timeSinceLastFiring > minFireInterval) {
                handReloaded = true;
            }
        } else {
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
//                        VortexUtils.moveMotorByEncoder(motorHand,
//                                fireP + handFireOvershotOffset + handFirePositionOffset,
//                                handFirePower);
                            motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            motorHand.setPower(handFirePower);
                            if (currentHandP - fireP > 0) {
                                fireState = 1; // go to reload state
                            }
                        }
                    }
                    break;
                case 1:
                    VortexUtils.moveMotorByEncoder(motorHand,
                            fireP, handFirePower * 0.5);
                    if (Math.abs(currentHandP - fireP) <= leftHandFirePositionTolerance
                            || timeSinceLastFiring > minFireInterval) {
                        fireState = 2;
                    }
                case 2:
                default:
                    // assume that reload is done
                    handReloaded = true;
                    fireState = 3;
                    break;
            }
        }
    }

    protected boolean hasReachedPosition ( int targetPos) {
        if ( Math.abs(motorArm.getCurrentPosition() - targetPos) < leftArmPositionTolerance) {
            return true;
        }
        return false;
    }
    
    public void shoot2 (boolean triggerOn) {
        long currentT = System.currentTimeMillis();
        if (triggerOn) {
            // fire only when the arm is in fire-safe zone
            int currentArmP = motorArm.getCurrentPosition();
            if (currentArmP > leftArmFiringSafeZone) {
                int fireP = getNextFirePosition();
                int current = motorHand.getCurrentPosition();
                if (handReloaded && currentT - lastFireTimeStamp < minFireInterval) {
                    // fire
                    fireCount++;
                    lastFireTimeStamp = currentT;

                    // encoder mode is slow. just use power mode.
                    motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorHand.setPower(handFirePower);
                    handReloaded = false;

                } else if ( current > fireP) {
                    // reload
                    VortexUtils.moveMotorByEncoder(motorHand,
                            fireP, handFirePower * 0.5);
                } else if (Math.abs(current - fireP) < leftHandFirePositionTolerance){
                    // reload is done
                    handReloaded = true;
                }
            }
        } else if (currentT - lastFireTimeStamp > minFireInterval){
            VortexUtils.moveMotorByEncoder(motorHand,
                    getNextFirePosition(),
                    handHoldPower);
            handReloaded = true;
        }
    }

    protected int getNextFirePosition () {
        int nexPos = handHomePosition + fireCount * handFirePositionOffset;
        reporter.addData("Particle shooter", "fire count %2d", fireCount);
        reporter.addData("Particle shooter", "hand pos   %2d", nexPos);
        return nexPos;
    }
}
