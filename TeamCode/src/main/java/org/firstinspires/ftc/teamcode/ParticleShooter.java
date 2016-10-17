package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ParticleShooter extends Excecutor {

    // arm
    protected int armStartPosition =0;
    protected int armFiringPosition1 = 4000;
    protected int armFiringPosition2 = 4000;
    protected double armPower = 1.0;
    protected int leftArmFiringSafeZone = 2000;

    // hand
    int fireState =0;
    protected int handHomePosition =0;
    protected int handFirePositionOffset = 560;
    protected int leftHandFireOvershotOffset = 350;
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
    }

    public int loop (int startState, int endState) {
        switch (state) {
            case 0:
                // move arm to firing position
                VortexUtils.moveMotorByEncoder(motorArm, armFiringPosition1, armPower);
                if ( motorArm.getCurrentPosition() == armFiringPosition1) {
                    fireState = 1;
                    state = 1;
                }
                break;
            case 1:
                // shoot the first particle
                fireLoop();
                if (fireState != 1) {
                    state = 2;
                }
                break;
            case 2:
                // adjust arm position
                VortexUtils.moveMotorByEncoder(motorArm, armFiringPosition2, armPower);
                if ( motorArm.getCurrentPosition() == armFiringPosition2) {
                    fireState = 1;
                    state = 3;
                }
                break;
            case 3:
                // shoot the second particle
                fireLoop();
                if (fireState != 1) {
                    state = 4;
                }
                break;
            case 4:
                // move arm back 
                VortexUtils.moveMotorByEncoder(motorArm, armStartPosition, armPower);
                break;
            default:
                return endState;
        }
        return startState;
    }
    
    public void fireLoop () {
        switch (fireState) {
            // firing trigger
            case 1:
                long currentT = System.currentTimeMillis();
                long timeSinceLastFiring = System.currentTimeMillis() - lastFireTimeStamp;
                int currentP = motorArm.getCurrentPosition();
                int fireP = handHomePosition + fireCount * handFirePositionOffset;
                if (leftHandReloaded
                        && timeSinceLastFiring > minFireInterval) {
                    if (currentP > leftArmFiringSafeZone) {
                        // fire
                        fireCount++;
                        lastFireTimeStamp = currentT;
                        VortexUtils.moveMotorByEncoder(motorHand,
                                fireP + leftHandFireOvershotOffset + handFirePositionOffset,
                                leftHandFirePower);
                    }
                } else if (timeSinceLastFiring > minReloadInterval) {
                    // reload
                    VortexUtils.moveMotorByEncoder(motorHand,
                            fireP, leftHandFirePower * 0.5);
                } else if (Math.abs(currentP - fireP) < leftHandFirePositionTolerance) {
                    // assume that reload is done
                    leftHandReloaded = true;
                    fireState = 0;
                }
                break;
            default:
                VortexUtils.moveMotorByEncoder(motorHand,
                        handHomePosition + fireCount * handFirePositionOffset,
                        leftHandHoldPower);
                leftHandReloaded = true;
        }
    }
}
