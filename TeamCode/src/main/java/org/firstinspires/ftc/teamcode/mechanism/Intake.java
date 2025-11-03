package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public DcMotor intakeMotor;
    public Servo hardStop;

    public boolean hardStopActivated = false;

    public boolean turretSpinIntake = false;

    public boolean spinIntake = false;
    public boolean reverseIntake = false;
    public double speed = .9;

    public void init(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardStop = hwMap.get(Servo.class, "stop");


    }

    public void setServoPos(double angle){
        hardStop.setPosition(angle);
    }


    public void hardStopPos(boolean activated){
        if(activated){
                setServoPos(.6);
        } else {
                setServoPos(.35);
            }
        }

    public void spin() {
        if(turretSpinIntake) {
            speed = 1;
            intakeMotor.setPower(-speed);
        } else if(reverseIntake){
            speed = .5;
            intakeMotor.setPower(speed);
        } else if (spinIntake) {
            intakeMotor.setPower(-speed);
        } else {
            intakeMotor.setPower(0);
        }
    }
}
