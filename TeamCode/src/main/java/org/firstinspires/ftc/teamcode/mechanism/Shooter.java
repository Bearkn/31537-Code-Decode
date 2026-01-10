package org.firstinspires.ftc.teamcode.mechanism;

import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Algs.PIDF;


public class Shooter {

    //shooter obj
    public DcMotorEx fly1;
    public DcMotorEx fly2;

    public Servo hood;

    //shooter variables

    public double Kp=0.00075,Ki = 0,Kd = 0,Kf=0.00045;

    public double targetFlywheelSpeed = 1600;

    public double currentFlywheelSpeed = 0;

    public double power;

    public double hoodAngle = .5;

    public double [] stepsizes = {.1,.01,0.001,.0001,.00001};

    public int stepIndex = 1;

    public boolean shooterActivated = false;

    //hood and rpm control

    public double minFlywheelSpeed = 1250;
    public double maxFlywheelSpeed = 2150;



    public void init(HardwareMap hwMap){
        fly1 = hwMap.get(DcMotorEx.class, "fly1");
        fly2 = hwMap.get(DcMotorEx.class, "fly2");
        hood = hwMap.get(Servo.class,"hood");
        fly1.setDirection(DcMotorSimple.Direction.REVERSE);
        fly1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fly1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fly2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void UpdateHoodAngle() {
        double clampedPosition = MathFunctions.clamp(hoodAngle, 0.3, 1.0);
        hood.setPosition(clampedPosition);
    }

    public void updateFLywheelSpeed(double distance){

//        double flyspeed = 4.49843*(distance) +1290.12596;


            double flyspeed =  (((0.00000777412 * distance
                    - 0.00254502) * distance
                    + 0.25954) * distance
                    - 3.51534) * distance
                    + 1333.01601;



        targetFlywheelSpeed =  MathFunctions.clamp(flyspeed,minFlywheelSpeed,maxFlywheelSpeed);

    }

    public void updateHoodAngle(double distance, double currentFlySpeed){

        double hoodangle = -19.0939552
                + 0.0008866700 * distance
                + 0.0305236838 * currentFlySpeed
                - 0.0000156994384 * currentFlySpeed * currentFlySpeed
                + 0.0000000027145231 * currentFlySpeed * currentFlySpeed * currentFlySpeed;

        hoodAngle =  MathFunctions.clamp(hoodangle,.3,1.0);
    }

    public static double distance2D(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public double hoodcontrol(double x) {
        double[] xs = {
                45, 50, 60, 70, 80, 93.3323,
                100, 110, 120, 130, 140, 150, 160
        };

        double[] ys = {
                0.50, 0.57, 0.60, 0.73, 0.77, 0.73,
                0.70, 0.68, 0.68, 0.68, 0.74, 0.74, 0.71
        };

        // Clamp if outside range
        if (x <= xs[0]) return ys[0];
        if (x >= xs[xs.length - 1]) return ys[ys.length - 1];

        // Find interval and interpolate
        for (int i = 0; i < xs.length - 1; i++) {
            if (x >= xs[i] && x <= xs[i + 1]) {
                double t = (x - xs[i]) / (xs[i + 1] - xs[i]);
                return ys[i] + t * (ys[i + 1] - ys[i]);
            }
        }

        // Should never reach here
        return Double.NaN;
    }


    public void update(double distance, double currentfly){

        PIDF shooterPID = new PIDF(Kp, Ki, Kd, Kf);
//        updateHoodAngle(distance,currentfly);
        hoodAngle =  MathFunctions.clamp(hoodcontrol(distance),.3,1.0);
        updateFLywheelSpeed(distance);
        UpdateHoodAngle();
        currentFlywheelSpeed = fly2.getVelocity();
        power = shooterPID.calculate(targetFlywheelSpeed, currentFlywheelSpeed);

        if(shooterActivated) {
            fly1.setPower(power);
            fly2.setPower(power);
        } else {
            fly1.setPower(0);
            fly2.setPower(0);
        }
    }


}
