package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private DcMotor flyWheelMotor1,flyWheelMotor2, turretMotor;

    public void init(HardwareMap hwMap) {
        flyWheelMotor1 = hwMap.get(DcMotor.class, "fly1");
        flyWheelMotor2 = hwMap.get(DcMotor.class, "fly2");
        turretMotor = hwMap.get(DcMotor.class, "turret");
        flyWheelMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void spin(double speed){

        flyWheelMotor1.setPower(speed);
        flyWheelMotor1.setPower(speed);
    }

}
