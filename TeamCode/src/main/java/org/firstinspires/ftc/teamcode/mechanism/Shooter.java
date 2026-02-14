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

    public double Kp=0.0012 ,Ki = 0,Kd = 0,Kf=0.000415;

    public double targetFlywheelSpeed = 1600;

    public double currentFlywheelSpeed = 0;

    public double power;

    public double hoodAngle = .75;

    public double [] stepsizes = {.1,.01,0.001,.0001,.00001};

    public int stepIndex = 1;

    public boolean shooterActivated = false;

    //hood and rpm control

    public double minFlywheelSpeed = 1250;
    public double maxFlywheelSpeed = 2150;

    //updated shooter constants

    public double SCORE_HEIGHT = 39;
    public double SCORE_ANGLE = Math.toRadians(-30);
    public double PASS_THROUGH_RADIUS = 5;

    public double checkHoodAngle;

    public double checkFlySpeed;





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
        double clampedPosition = MathFunctions.clamp(hoodAngle, 0.5, 1.0);
        hood.setPosition(clampedPosition);
//        hood.setPosition(.85);

    }

//    public void updateFlywheelSpeed(double distance) {
//
//        double flyspeed;
//
//        if (distance < 130) {
//            // Old regression
//            flyspeed = 0.00000770804 * Math.pow(distance, 4)
//                    - 0.00385759 * Math.pow(distance, 3)
//                    + 0.675656 * Math.pow(distance, 2)
//                    - 39.70454 * distance
//                    + 2329.88133
//                    + 40;
//        } else {
//            // New regression
//            flyspeed = -0.25 * distance * distance
//                    + 82.5 * distance
//                    - 4340+100;
//        }
//
//        targetFlywheelSpeed = flyspeed;
//    }

    public void updateFlywheelSpeed(double distance) {

        double flyspeed =
                0.00000549971 * Math.pow(distance, 4)
                        - 0.00276458 * Math.pow(distance, 3)
                        + 0.489946 * Math.pow(distance, 2)
                        - 27.20115 * distance
                        + 2090.09518+20;

        targetFlywheelSpeed = flyspeed;
    }




//    public void updateHoodAngle(double distance, double currentFlySpeed){
//
//        double hoodangle = -19.0939552
//                + 0.0008866700 * distance
//                + 0.0305236838 * currentFlySpeed
//                - 0.0000156994384 * currentFlySpeed * currentFlySpeed
//                + 0.0000000027145231 * currentFlySpeed * currentFlySpeed * currentFlySpeed;
//
////        hoodAngle =  MathFunctions.clamp(hoodangle,.3,1.0);
////        hoodAngle = .5;
//    }
//    public void updateHoodAngle(double distance, double currentFlySpeed) {
//
//        double hoodAngle =
//                0.762934 /
//                        (1.0 + Math.exp(-(0.173309 * distance - 6.2873)));
//
//         hoodAngle = MathFunctions.clamp(hoodAngle, 0.3, 1.0);
//
//    }

    public static double distance2D(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return Math.sqrt(dx * dx + dy * dy);
    }

//    public double hoodcontrol(double x) {
//        // Using Horner's method for efficient polynomial evaluation
//        double result = (-4.20309e-8 * x + 0.0000164867) * x;
//        result = (result - 0.00235657) * x;
//        result = (result + 0.146512) * x;
//        result = result - 2.56355;
//
//        return result;
//    }
//    public double hoodcontrol(double x) {
//
//        if (x < 130) {
//            return (0.762934 / (1.0 + Math.exp(-(0.173309 * x - 6.2873)))) + .02;
//        } else {
//            return .65;
//        }
//    }

    public double hoodControl(double x) {
        if (x >= 0 && x <= 80) {
            return 0.00000916667 * Math.pow(x, 3)
                    - 0.00175 * Math.pow(x, 2)
                    + 0.116083 * x
                    - 1.929;
        }
        else if (x > 80 && x <= 120) {
            return 0.85;
        }
        else { // x > 120
            return 0.005 * x + 0.3;
        }
    }





    public void update(double distance, double currentfly){

        PIDF shooterPID = new PIDF(Kp, Ki, Kd, Kf);
        hoodAngle = MathFunctions.clamp(hoodControl(distance), .5, 1.0);
        updateFlywheelSpeed(distance);
        UpdateHoodAngle();
        currentFlywheelSpeed = fly2.getVelocity();
        power = shooterPID.calculate(targetFlywheelSpeed, currentFlywheelSpeed);
        power = MathFunctions.clamp(power, -.2,1);
        if(shooterActivated) {
//            targetFlywheelSpeed = 1600;
            fly1.setPower(power);
            fly2.setPower(power);
        } else {
//            targetFlywheelSpeed = 2000;
            fly1.setPower(0);
            fly2.setPower(0);
        }
    }




    /// // updated shooter

    public double mapDegreesToRange(double degrees) {
        return 0.5 + ((degrees - 20.0) / (45.0 - 20.0)) * (1.0 - 0.5);
    }

    public double getExitVelocity(double x) {
        return MathFunctions.clamp(0.0000155458 * x * x + 0.0555902 * x + 47.69158,100,285);
    }

    public static double getXFromY(double y) {
        return (-0.0555902 + Math.sqrt(
                0.0555902 * 0.0555902
                        - 4 * 0.0000155458 * (47.69158 - y)
        )) / (2 * 0.0000155458);
    }


    public void updatedShooter(double distance) {

        double g = 32.174 * 12;
        double x = distance - PASS_THROUGH_RADIUS;
        double y = SCORE_HEIGHT;
        double a = SCORE_ANGLE;

        double hoodangle = MathFunctions.clamp(Math.atan(2*y/x - Math.tan(a)),.5,1);

        double flySpeed = Math.sqrt(g*x*x/(2*Math.pow(Math.cos(hoodangle),2)*(x*Math.tan(hoodangle)-y)));

//        double vz = flySpeed * Math.sin(hoodangle);
//        double time = x/ (flySpeed * Math.cos(hoodangle));

        hoodAngle = hoodangle;
        targetFlywheelSpeed = getXFromY(flySpeed);
    }




}
