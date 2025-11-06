package org.firstinspires.ftc.teamcode.mechanism;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Vision {

    public Limelight3A limelight;
    MecanumDrive mecanumDrive = new MecanumDrive();
    public LLResult llResult;

    public void init(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); //april tag 24
        limelight.start();



    }

    public void loop(){
//        YawPitchRollAngles orientation = mecanumDrive.imu.();
//        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        llResult = limelight.getLatestResult();


//            telemetry.addData("bot pose", botPoseMt2);


        
    }
}
