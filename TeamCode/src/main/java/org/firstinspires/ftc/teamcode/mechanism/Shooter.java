//package org.firstinspires.ftc.teamcode.mechanism;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//
//import org.firstinspires.ftc.teamcode.Algs.PIDF;
//
//@TeleOp
//public class Shooter extends OpMode {
//
//    //shooter obj
//    public DcMotorEx fly1;
//    public DcMotorEx fly2;
//
//    //shooter variables
//
//    public double Kp,Ki = 0,Kd = 0,Kf;
//
//    public double targetFlywheelSpeed = 1600;
//
//    private double currentFlywheelSpeed = 0;
//
//    double [] stepsizes = {10.0,1.0,0.1,.01,.001};
//
//    int stepIndex = 1;
//
//
//    PIDF shooterPID = new PIDF(Kp, Ki, Kd, Kf);
//
//    public void init(){
//        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
//        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
//        fly1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        fly2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(Kp,Ki,Kd,Kf);
//        telemetry.addLine("init complete");
//    }
//
//    public void loop(){
//        currentFlywheelSpeed = fly1.getVelocity();
//        double power = shooterPID.calculate(targetFlywheelSpeed, currentFlywheelSpeed);
//        fly1.setPower(power);
//        fly2.setPower(power);
//    }
//}
