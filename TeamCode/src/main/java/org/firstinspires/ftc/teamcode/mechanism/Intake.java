package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public DcMotor intakeMotor;
    public Servo hardStop;

    public boolean spinIntake = false;
    public double speed = 1;

    public void init(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardStop = hwMap.get(Servo.class, "stop");


    }

    public void setServoPos(double angle){
        hardStop.setPosition(angle);
    }

    public void spin() {
        if (spinIntake == true) {
            intakeMotor.setPower(-speed);
        } else {
            intakeMotor.setPower(0);
        }
    }
}
