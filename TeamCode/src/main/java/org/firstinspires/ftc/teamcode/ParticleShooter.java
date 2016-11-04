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
    protected int handFirePositionOffset = 450; // 20: 1 motor is 560. 16:1 is 450
    protected int leftHandFireOvershotOffset = 230; // 20:1 motor is 350. 16:1 is 230
    protected double leftHandHoldPower = 0.05;
    protected double leftHandFirePower = 1.0;
    protected int fireCount =0;
    protected long lastFireTimeStamp = 0;
    protected long minFireInterval = 1500;
    protected long minReloadInterval = 1000;
    protected boolean leftHandReloaded = true;
    protected int leftHandFirePositionTolerance = 10;
    
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
                if (leftHandReloaded
                        && timeSinceLastFiring > minFireInterval) {
                    if (currentP > leftArmFiringSafeZone) {
                        // fire
                        fireCount++;
                        lastFireTimeStamp = currentT;
                        VortexUtils.moveMotorByEncoder(motorHand,
                                fireP + leftHandFireOvershotOffset + handFirePositionOffset,
                                leftHandFirePower);
                        fireState = 1; // go to reload state
                    }
                }
                break;
            case 1:
                // reload
                if (timeSinceLastFiring > minReloadInterval) {
                    // reload
                    VortexUtils.moveMotorByEncoder(motorHand,
                            fireP, leftHandFirePower * 0.5);
                    fireState = 2;
                }
            case 2:
                if (Math.abs(currentP - fireP) < leftHandFirePositionTolerance) {
                    // assume that reload is done
                    leftHandReloaded = true;
                    fireState = 3;
                }
                break;
            case 3:
            default:
                // settle to ready position
                VortexUtils.moveMotorByEncoder(motorHand,
                        handHomePosition + fireCount * handFirePositionOffset,
                        leftHandHoldPower);
                leftHandReloaded = true;
                break;
        }
    }

    protected boolean hasReachedPosition ( int targetPos) {
        if ( Math.abs(motorArm.getCurrentPosition() - targetPos) < leftArmPosotionTolerance ) {
            return true;
        }
        return false;
    }
}
