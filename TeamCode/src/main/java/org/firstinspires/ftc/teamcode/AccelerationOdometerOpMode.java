package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
/**
 * Created by andrewwang on 3/12/17.
 */

public class AccelerationOdometerOpMode extends VortexTeleOp{
    HardwareGyroTracker GTH = null;
    AccelerationOdometer AO = null;
    @Override
    public void init(){
        GTH = new HardwareGyroTracker();
        GTH.init(hardwareMap);
        super.init();
        AO= new AccelerationOdometer(GTH.gyro);

    }
    @Override
    public void init_loop() {
        super.init_loop();
                    // make sure the gyro is calibrated.
            if (GTH.gyro.isCalibrating())  {
                telemetry.addData("Gyro measuring mode", GTH.gyro.getMeasurementMode());
            telemetry.addData(">", "Gyro is calibrating.  DO NOT start!!!!");
        }
        else {
            telemetry.addData(">", "Press Start.");
        }
    }


}
