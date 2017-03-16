package org.firstinspires.ftc.teamcode;

import java.lang.Math;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;

/**
 * dfpispduf83uh8793fj9pdaisdfh012jdpa9808h3pq89d0aisd701u23-27duoha=21y89f=-12udf0-+u3207dfUI28@7DA8923HFADA9SUEHFAPS9DF0Q34LDjasdu90f7h   h98as0df+_+hfo8dasefgsahdfbe87
 * aosd8gfuasi78*(&)7dfaogs+)9d8fa97s9e0ras98euraygsdofe)*()ogydas6e97gyfvdashe8hyfgda7s8e_f76gd8s7ge0-a=se9ifu8ds-f=daos8f=d-sdf-e-sao8sd7f9aesa-df78e9s
 * asodf-8eu078a0-21y7831=9-13278yr9_+)+)(__)(*U&Y38127307_7321927ytgdAGDFYUsyehfuYGHAdhuagsdifuyhaUGyufyhsduyfgayusdgfYgIYgYUGiyukfaeisa678123
 * Created by andrewwang on 1/15/17.aodfoa87sefg0123-9&*Ady78siuy7e8&o2ql,df7e8a.sdf78o.Qdoua7819eaj98&Auds87df*^Wyhdf78aosCancerasoud9u320wdrFATewasd8pfaseadchv
 * udas80df9haspd98fp
 */
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.robotcore.hardware.DcMotor;
public class AccelerationOdometer extends Tracker{
    public int tick = 50;
    ModernRoboticsI2cCompassSensor compass = null;
    ModernRoboticsI2cGyro gyro = null;
    private long lasttimestamp = 0;
    private int state = 0;
    private DcMotor leftWheel = null;
    private DcMotor rightWheel = null;
    protected double x = 0;
    protected double y = 0;
    protected double angle = 0;
    protected double distance = 0;
    protected double xspeed = 0;
    protected double yspeed = 0;
    protected Acceleration a;
    protected double xa = 0;
    protected double ya = 0;
    private double d;
    public AccelerationOdometer(ModernRoboticsI2cGyro g, DcMotor leftW,
                                DcMotor rightW, ModernRoboticsI2cCompassSensor c){
        gyro = g;
        compass = c;
        leftWheel = leftW;
        rightWheel = rightW;
    }
    public void loop() {
        switch(state){
            case 1:
                distance = 0;
                lasttimestamp = System.currentTimeMillis();
                //set time stamp
                state = 2;
                //wait until tick
                if (lasttimestamp <= System.currentTimeMillis() - tick){
                    state = 3;
                }
                break;
            case 3:
                //Get angle
                angle = gyro.getHeading();
                state = 4;
                break;
            case 4:
                //get acceleration
                a = compass.getAcceleration();
                xa = a.xAccel;
                ya = a.yAccel;
                xspeed = calculatespeed(xa, xspeed);
                yspeed = calculatespeed(ya, yspeed);
                state = 5;
            case 5:
                x += xspeed/1000*tick;
                y += yspeed/1000*tick;
            default:
                state = 1;
                break;
        }
    }
    private double calculatespeed(double ac, double speed){
        speed += ac;
        return speed/1000*tick;
    }
    protected void drive(double speed){
        leftWheel.setPower(speed);
        rightWheel.setPower(speed);
    }
    public boolean drive(float distance){
        double tempx = x;
        double tempy = y;
        double destx;
        double desty;
        double tempa = angle;
        double pied = torad(angle);
        double farx = Math.cos(pied)*distance;
        double fary = Math.sin(pied)*distance;
        destx = tempx + farx;
        desty = tempy + fary;
        if(destx+1 >= x && desty+1 >= y && destx - 1 <= x && desty - 1 <= y){
            drive(0);
            return true;
        }else if (angle == tempa){
            drive(0.5);
        }else{
            if((angle - tempa) < 0){
                leftWheel.setPower(0.05);
                rightWheel.setPower(-0.05);
            }else{
                leftWheel.setPower(-0.05);
                rightWheel.setPower(0.05);
            }
        }
        return false;
    }
    public double torad(double a){
        return a*Math.PI/180;
    }

}
