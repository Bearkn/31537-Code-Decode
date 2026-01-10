package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Algs.PIDF;

public class Intake {

    //Intake obj
    public DcMotorEx intakef;
    public DcMotorEx index;

    public Servo stop;

    //Intake Variables
    public boolean IntakeActivated = true;

    public boolean teleOpIntake = true;

    public boolean intakeOn = false;

    public boolean stopOn = true;

    public boolean Outtake = false;


    public enum StopState {
        SHOOT,
        HOLD
    }
    public StopState stopState;

    public enum IntakeState {
        INTAKE,
        STOP,
        OUTTAKE
    }
    public IntakeState intakeState;

    public enum IndexState {
        INTAKE,
        STOP,
        OUTTAKE
    }
    public IndexState indexState;



    public double targetIntakeSpeed;
    public double currentIntakeSpeed;

    public double Kp=.0005,Ki = 0,Kd = 0,Kf=2.8;
    public double power;


    public double indexSpeed;

    public void init(HardwareMap hwMap){
        intakef = hwMap.get(DcMotorEx.class, "intakef");
        index = hwMap.get(DcMotorEx.class, "index");
        stop = hwMap.get(Servo.class,"stop");
        intakef.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        index.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakef.setDirection(DcMotorSimple.Direction.REVERSE);
        index.setDirection(DcMotorSimple.Direction.REVERSE);
        intakef.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        index.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        stopState = StopState.HOLD;

    }

    public void update(){
        if(IntakeActivated) {
            StopUpdate();
            IntakeUpdate();
            IndexUpdate();
            if (teleOpIntake) {
                PIDF intakePID = new PIDF(Kp, Ki, Kd, Kf);
                currentIntakeSpeed = intakef.getVelocity();
                power = intakePID.calculate(targetIntakeSpeed, currentIntakeSpeed);
                intakef.setPower(power);
                index.setPower(indexSpeed);
            }
        }
    }

    public void StopUpdate() {
        switch (stopState) {
            case SHOOT:
                stop.setPosition(.1);
                break;
            case HOLD:
                stop.setPosition(.5);
                break;
        }
    }

    public void IntakeUpdate() {
        switch (intakeState) {
            case INTAKE:
                targetIntakeSpeed = 1;
                break;
            case STOP:
                targetIntakeSpeed = 0;
                break;
            case OUTTAKE:
                targetIntakeSpeed = -.5;
                break;
        }
    }

    public void IndexUpdate() {
        switch (indexState) {
            case INTAKE:
                indexSpeed = 1;
                break;
            case STOP:
                indexSpeed = .0;
                break;
            case OUTTAKE:
                indexSpeed = -.5;
                break;
        }
    }
}
