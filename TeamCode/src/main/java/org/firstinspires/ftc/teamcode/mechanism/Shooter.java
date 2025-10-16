package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private DcMotor flyWheelMotor;

    public void init(HardwareMap hwMap) {
        flyWheelMotor = hwMap.get(DcMotor.class, "flywheel");
        flyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void spin(double speed){
        flyWheelMotor.setPower(speed);
    }

}
