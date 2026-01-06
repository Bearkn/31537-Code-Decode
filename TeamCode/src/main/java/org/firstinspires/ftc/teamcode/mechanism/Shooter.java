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

    public double targetFlywheelSpeed = 0;

    public double currentFlywheelSpeed = 0;

    public double power;

    public double hoodAngle = .5;

    public double [] stepsizes = {.1,.01,0.001,.0001,.00001};

    public int stepIndex = 1;

    public boolean shooterActivated = false;



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
    }

    public double updateFLywheelSpeed(double distance){

        return MathFunctions.clamp(distance,0,2200);

    }

    public double updateHoodAngle(double distance){

        return MathFunctions.clamp(distance,.5,1);
    }


    public void update(){

        PIDF shooterPID = new PIDF(Kp, Ki, Kd, Kf);
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
