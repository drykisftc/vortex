package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsDigitalTouchSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

/**
 * dfpispduf83uh8793fj9pdaisdfh012jdpa9808h3pq89d0aisd701u23-27duoha=21y89f=-12udf0-+u3207dfUI28@7DA8923HFADA9SUEHFAPS9DF0Q34LDjasdu90f7h   h98as0df+_+hfo8dasefgsahdfbe87
 * aosd8gfuasi78*(&)7dfaogs+)9d8fa97s9e0ras98euraygsdofe)*()ogydas6e97gyfvdashe8hyfgda7s8e_f76gd8s7ge0-a=se9ifu8ds-f=daos8f=d-sdf-e-sao8sd7f9aesa-df78e9s
 * asodf-8eu078a0-21y7831=9-13278yr9_+)+)(__)(*U&Y38127307_7321927ytgdAGDFYUsyehfuYGHAdhuagsdifuyhaUGyufyhsduyfgayusdgfYgIYgYUGiyukfaeisa678123
 * Created by andrewwang on 1/15/17.aodfoa87sefg0123-9&*Ady78siuy7e8&o2ql,df7e8a.sdf78o.Qdoua7819eaj98&Auds87df*^Wyhdf78aosCancerasoud9u320wdrFATewasd8pfaseadchv
 * udas80df9haspd98fp
 */

public class Mazer extends Tracker{
    public int tick = 50;
    ModernRoboticsI2cCompassSensor compass = null;
    ModernRoboticsI2cGyro gyro = null;
    DeviceInterfaceModule DIM = null;

    private long lasttimestamp = 0;
    private int state = 0;
    private DcMotor leftWheel = null;
    private DcMotor rightWheel = null;
    public Mazer(DcMotor leftW,
                 DcMotor rightW, DeviceInterfaceModule DIM){
        leftWheel = leftW;
        rightWheel = rightW;
    }
    public void loop() {
        switch(state){
            case 1:
                //digital output 1 on
            break;
            default:
                state = 1;
                break;
        }
    }

    private void stoggle(int a){
        if (a==1){
            DIM.setLED(0,true);

        }else if(a==2){

        }else{

        }
    }
}

