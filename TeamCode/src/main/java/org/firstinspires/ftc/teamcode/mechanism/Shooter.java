package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    public DcMotor flyWheelMotor1,flyWheelMotor2, turretMotor;

    public boolean spinShooter = false;
    public double speed = 0.5;

    public void init(HardwareMap hwMap) {
        flyWheelMotor1 = hwMap.get(DcMotor.class, "fly1");
        flyWheelMotor2 = hwMap.get(DcMotor.class, "fly2");
        turretMotor = hwMap.get(DcMotor.class, "turret");
        flyWheelMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void spin(){
        if(spinShooter == true){
            flyWheelMotor1.setPower(speed);
            flyWheelMotor2.setPower(-speed);
        } else {
            flyWheelMotor1.setPower(0);
            flyWheelMotor2.setPower(0);
        }
//
    }

}
