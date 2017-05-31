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
    protected Acceleration acc;
    protected double xa = 0;
    protected double ya = 0;
    private double d;
    private int tolerance = 3;
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
            case 2:
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
                acc = compass.getAcceleration();
                xa = acc.xAccel;
                ya = acc.yAccel;
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
    //calculate speed form acceleration and current speed
    private double calculatespeed(double ac, double speed){
        speed += ac/1000*tick;
        return speed;
    }
    //drive function
    protected void drive(double speed){
        leftWheel.setPower(speed);
        rightWheel.setPower(speed);
    }
    //loop for drive a certian distance
    public boolean dloop(double tempx, double tempy){
        double destx;
        double desty;
        double tempa = angle;
        double pied = torad(angle);
        double farx = Math.cos(pied)*distance;
        double fary = Math.sin(pied)*distance;
        destx = tempx + farx;
        desty = tempy + fary;
        if(destx+tolerance >= x && desty+tolerance >= y && destx - tolerance <= x && desty - tolerance <= y){
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
    //to initialize dloop
    public boolean driveforward(double distance){
        double tempx = x;
        double tempy = y;
        while(true) {
            if(dloop(tempx, tempy)){
                return true;

            }
        }
    }
    //to radians to degrees
    public double torad(double a){
        return a*Math.PI/180;
    }
    //from radians to degrees
    public double fromrad(double v){
        return v*180/Math.PI;
    }
    //just like gyro.turn
    private boolean turn(double a) {
        if (gyro.getHeading() < a + 1 && gyro.getHeading() > a - 1) {
            return true;
        } else if ((angle - a) < 0) {
            leftWheel.setPower(0.05);
            rightWheel.setPower(-0.05);
        } else {
            leftWheel.setPower(-0.05);
            rightWheel.setPower(0.05);
        }
        return false;
    }
    //loop for moving to certian location
    public boolean mloop(double tempx, double tempy,double x1, double y1){
        double pied = Math.atan(tempy/tempx);
        double angle = fromrad(pied);
        if (x < x1 + tolerance&& x1 - tolerance < x && y < y1 + tolerance&& y1 - tolerance < y){
            return true;
        }
        if (turn(angle)){
            driveforward(Math.hypot(tempx, tempy));
        }
        //loop
        return false;
    }
    //mloop initialization
    public boolean moveto(double x1, double y1){
        double tempx = x;
        double tempy = y;
        while(true){
            if(mloop(tempx,tempy,x1,y1)){
                return true;
            }
        }

    }

}
