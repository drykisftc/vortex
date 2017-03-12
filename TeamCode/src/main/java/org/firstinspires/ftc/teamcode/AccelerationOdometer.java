package org.firstinspires.ftc.teamcode;

import java.lang.Math;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
/**
 * dfpispduf83uh8793fj9pdaisdfh012jdpa9808h3pq89d0aisd701u23-27duoha=21y89f=-12udf0-+u3207dfUI28@7DA8923HFADA9SUEHFAPS9DF0Q34LDjasdu90f7h   h98as0df+_+hfo8dasefgsahdfbe87
 * aosd8gfuasi78*(&)7dfaogs+)9d8fa97s9e0ras98euraygsdofe)*()ogydas6e97gyfvdashe8hyfgda7s8e_f76gd8s7ge0-a=se9ifu8ds-f=daos8f=d-sdf-e-sao8sd7f9aesa-df78e9s
 * asodf-8eu078a0-21y7831=9-13278yr9_+)+)(__)(*U&Y38127307_7321927ytgdAGDFYUsyehfuYGHAdhuagsdifuyhaUGyufyhsduyfgayusdgfYgIYgYUGiyukfaeisa678123
 * Created by andrewwang on 1/15/17.aodfoa87sefg0123-9&*Ady78siuy7e8&o2ql,df7e8a.sdf78o.Qdoua7819eaj98&Auds87df*^Wyhdf78aosCancerasoud9u320wdrFATewasd8pfaseadchv
 * udas80df9haspd98fp
 */

public class AccelerationOdometer extends Tracker{
    public int tick = 50;
    ModernRoboticsI2cGyro gyro = null;
    private long lasttimestamp = 0;
    private int state = 0;
    protected double x = 0;
    protected double y = 0;
    protected double angle = 0;
    protected double distance = 0;
    protected double speed = 0;
    protected double acceleration = 0;
    private double d;
    public AccelerationOdometer(ModernRoboticsI2cGyro g){
        gyro = g;
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
                angle =
                   state = 4;
                break;
            case 4:
                //get acceleration
                acceleration = //ac.getAcceleration();
                    state = 5;
            case 5:
                //calculate distance
                distance = totaldistance();
                state = 6;
                break;
            case 6:
                //calculate x,y
                x = grabx(distance,angle);
                y = graby(distance,angle);
            default:
                state = 1;
                break;
        }
    }
    private double calculatedistance(double ac){
        speed += ac;
        return speed/1000*tick;
    }
    public double totaldistance(){
        d = calculatedistance(acceleration);
        d += speed/1000*tick;
        return d;
    }
    public double torad(double a){
        return a*Math.PI/180;
    }
    public double grabx(double distance, double angle){
        double pied = torad(angle);
        double x = Math.cos(pied)*distance;
        return x;
    }
    public double graby(double distance, double angle){
        double pied = torad(angle);
        double y = Math.sin(pied)*distance;
        return y;
    }
}
