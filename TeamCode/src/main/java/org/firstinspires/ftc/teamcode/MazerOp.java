package org.firstinspires.ftc.teamcode;

/**
 * Created by andrewwang on 3/12/17.
 */

public class MazerOp extends VortexTeleOp{
    HardwareGyroTracker GTH = null;
    Mazer AO = null;
    HardwareCompass HC = null;
    boolean temp = true;
    int dist = 1000;
    private int state = 0;
    @Override
    public void init(){
        GTH = new HardwareGyroTracker();
        GTH.init(hardwareMap);
        HC = new HardwareCompass();
        HC.init(hardwareMap);

        super.init();
        AO= new Mazer(robot.motorLeftWheel,
                robot.motorRightWheel, );


    }
    @Override
    public void init_loop() {
        super.init_loop();
                    // make sure the gyro is calibrated.
            if (GTH.gyro.isCalibrating())  {
            telemetry.addData(">", "Gyro is calibrating.  DO NOT start!!!!");
        }
        else if(!HC.Compass.isCalibrating()) {
            telemetry.addData(">", "Press Start.");
        }
            if (HC.Compass.isCalibrating()) {
                telemetry.addData(">", "Compass is calibrating, DO NOT START");
            }
    }
    @Override
    public void start() {
        super.start();
        // compute baseline brightness
        AO.start(0);
        raiseArm();
        state = 0;
    }
    public void raiseArm () {
        VortexUtils.moveMotorByEncoder(robot.motorLeftArm, leftArmMovePosition, leftArmAutoMovePower);

    }
    @Override
    public void loop(){

    }
    }
