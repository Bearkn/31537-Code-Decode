package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Algs.PIDF;

public class Intake {

    //Intake obj
    public static DcMotorEx intakef;
    public static DcMotorEx index;

    public static Servo stop;

    //Intake Variables
    public static boolean IntakeActivated = true;

    public static boolean teleOpIntake = true;

    public static boolean intakeOn = false;

    public static boolean stopOn = false;

    public static boolean Outtake = false;


    public static enum StopState {
        SHOOT,
        HOLD
    }
    public static StopState stopState;

    public static enum IntakeState {
        INTAKE,
        STOP,
        OUTTAKE
    }
    public static IntakeState intakeState;

    public static enum IndexState {
        INTAKE,
        STOP,
        OUTTAKE
    }
    public static IndexState indexState;



    public static double intakeSpeed;
    public static double indexSpeed;

    public static void init(HardwareMap hwMap){
        intakef = hwMap.get(DcMotorEx.class, "intakef");
        index = hwMap.get(DcMotorEx.class, "index");
        stop = hwMap.get(Servo.class,"stop");
        intakef.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        index.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakef.setDirection(DcMotorSimple.Direction.REVERSE);
        index.setDirection(DcMotorSimple.Direction.REVERSE);
        intakef.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        index.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        stopState = StopState.HOLD;
    }

    public static void update(){
        if(IntakeActivated) {
            StopUpdate();
            if (teleOpIntake) {
                intakef.setPower(intakeSpeed);
                index.setVelocity(indexSpeed);
            }
        }
    }

    public static void StopUpdate() {
        switch (stopState) {
            case SHOOT:
                stop.setPosition(.0);
                break;
            case HOLD:
                stop.setPosition(.5);
                break;
        }
    }

    public static void IntakeUpdate() {
        switch (intakeState) {
            case INTAKE:
                intakeSpeed = .5;
                break;
            case STOP:
                intakeSpeed = 0;
                break;
            case OUTTAKE:
                intakeSpeed = -.5;
                break;
        }
    }

    public static void IndexUpdate() {
        switch (indexState) {
            case INTAKE:
                indexSpeed = .5;
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
