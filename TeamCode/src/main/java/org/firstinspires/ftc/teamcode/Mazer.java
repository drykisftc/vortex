package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsDigitalTouchSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
    HardwareWallTracker WT = null;
    private long lasttimestamp = 0;
    private int state = 0;
    private DcMotor leftWheel = null;
    private double Dist = 0;
    private double LTS = 0;
    private DcMotor rightWheel = null;
    private boolean rightSensor = false;
    private boolean frontSensor = false;
    private boolean leftSensor = false;
    private float ContactRange = 10;
    //Delay means time waited between sensor readings to avoid interference
    private int Delay = 20;


    public Mazer(DcMotor leftW,
                 DcMotor rightW, DeviceInterfaceModule DIM, HardwareWallTracker WT){
        leftWheel = leftW;
        rightWheel = rightW;
    }
    public void loop() {
        switch(state){
            //Cases 1,3,5 are turning on sensors and getting readings
            //Cases 2,4,6 are turning sensors off and waiting a bit of time to ensure no interference.
            case 1:
                stoggle(1);
                Dist = WT.sonicRange.getDistance(DistanceUnit.CM);
                LTS = System.currentTimeMillis();
                if(Dist < ContactRange){
                    rightSensor = true;
                }else{
                    rightSensor = false;
                }
            case 2:
                stoggle(4);
                if(LTS + Delay <= System.currentTimeMillis()){
                    state = 3;
                }
                break;
            case 3:
                stoggle(2);
                Dist = WT.sonicRange.getDistance(DistanceUnit.CM);
                LTS = System.currentTimeMillis();
                if(Dist < ContactRange){
                    frontSensor = true;
                }else{
                    frontSensor = false;
                }
            case 4:
                stoggle(4);
                if(LTS + Delay <= System.currentTimeMillis()){
                    state = 5;
                }
                break;
            case 5:
                stoggle(3);
                Dist = WT.sonicRange.getDistance(DistanceUnit.CM);
                LTS = System.currentTimeMillis();
                if(Dist < ContactRange){
                    leftSensor = true;
                }else{
                    leftSensor = false;
                }
            case 6:
                stoggle(4);
                if(LTS + Delay <= System.currentTimeMillis()){
                    state = 7;
                }
                break;
            default:
                state = 1;
                break;
        }
    }

    private void stoggle(int a){
        if (a==1){
            DIM.setLED(0,true);
            DIM.setLED(1,false);
            DIM.setLED(2,false);
        }else if(a==2){
            DIM.setLED(0,false);
            DIM.setLED(1,true);
            DIM.setLED(2,false);
        }else if (a==3){
            DIM.setLED(0,false);
            DIM.setLED(1,false);
            DIM.setLED(2,true);
        }else{
            DIM.setLED(1,false);
            DIM.setLED(0,false);
            DIM.setLED(2,false);
        }
    }
}

