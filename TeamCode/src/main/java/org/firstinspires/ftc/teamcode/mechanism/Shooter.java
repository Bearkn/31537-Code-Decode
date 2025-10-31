package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private DcMotor flyWheelMotor;
    private DCMotor turretMotor;

    public void init(HardwareMap hwMap) {
        flyWheelMotor = hwMap.get(DcMotor.class, "flywheel");
        turretMotor = hwMap.get(DcMotor.class, "turret");
        flyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void spin(double speed){
        flyWheelMotor.setPower(speed);
    }

}
