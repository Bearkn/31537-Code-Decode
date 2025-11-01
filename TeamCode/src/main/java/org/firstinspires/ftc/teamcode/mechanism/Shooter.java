package org.firstinspires.ftc.teamcode.mechanism;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Main;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Shooter {
    Intake intake = new Intake();
    private Follower follower;
    public DcMotorEx flyWheelMotor1,flyWheelMotor2, turretMotor;
//    public double distanceFromGoal;

    public boolean spinShooter = false;

    public boolean flyWheelActivated = false;

    public boolean shooting = false;


    public double speed = 1500;

    private final Pose startPose = new Pose(48, 96, Math.toRadians(0)); // Start Pose of our robot.


    double integralSum = 0;
    public double Kp = 0;
    public double Ki = 0;
    public double Kd = 0;

    public double Kf = 1;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

//    Main main = new Main();
    public void init(HardwareMap hwMap) {
        flyWheelMotor1 = hwMap.get(DcMotorEx.class, "fly1");
        flyWheelMotor2 = hwMap.get(DcMotorEx.class, "fly2");
        turretMotor = hwMap.get(DcMotorEx.class, "turret");
        flyWheelMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower = Constants.createFollower(hwMap);
        follower.setStartingPose(startPose);



    }

    public void spin(){
        if(spinShooter ){
            flyWheelMotor1.setVelocity(speed);
            flyWheelMotor2.setVelocity(-speed);
        } else {
            flyWheelMotor1.setPower(0);
            flyWheelMotor2.setPower(0);
        }
//
    }


    public double angleNormalize(int rotation) {
        double angle = (rotation / 10.0) % 360;
        if (angle > 180) {
            angle -= 360;
        } else if (angle <= -180) {
            angle += 360;
        }
        return angle;
    }
    public void speedCalc(double distance){
        speed = distance * 25;
    }

    public void spinTurret (double angle){
        angle = MathFunctions.clamp(angle, -90, 90);
        double error = angle - angleNormalize(turretMotor.getCurrentPosition());
        if (Math.abs(error) > -.5 ) {
            turretMotor.setPower(MathFunctions.clamp(error * .05, -1, 1));
        } else {
            turretMotor.setPower(0);
        }
    }

    public double turretAngle(boolean red) {
        follower.update();
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double robotTheta = follower.getPose().getHeading();
        Pose RobotPose = new Pose(robotX, robotY, Math.toDegrees(robotTheta));
        double angleTurret = 0;
        if (red) {
            Pose redGoal = new Pose(136, 136, Math.toDegrees(0)); // Start Pose of our robot.
            double distanceFromGoal = Math.sqrt(Math.pow(robotX - redGoal.getPose().getX(), 2) + Math.pow(robotY - redGoal.getPose().getY(), 2));

            double distFromGoalY = redGoal.getPose().getY() - robotY;

            double angleToGoal = -(Math.acos(distFromGoalY / distanceFromGoal) * (180 / Math.PI));

            angleTurret = angleToGoal - (robotTheta* (180 / Math.PI));

            speedCalc(distanceFromGoal);


        } else {
            Pose blueGoal = new Pose(8, 136, Math.toDegrees(0)); // Start Pose of our robot.

            double distanceFromGoal = Math.sqrt(Math.pow(robotX - 8, 2) + Math.pow(robotY - 136, 2));

            double distFromGoalY =136 - robotY;

            double angleToGoal = Math.acos(distFromGoalY / distanceFromGoal) * (180 / Math.PI);

            angleTurret = angleToGoal - (robotTheta* (180 / Math.PI));

            speedCalc(distanceFromGoal);

        }


        return angleTurret;
    }

//    public double PIDControl(double reference, double state){
//        double error = reference - state;
//        integralSum += error * timer.seconds();
//        double derivative = (error - lastError)/timer.seconds();
//        lastError = error;
//
//        timer.reset();
//
//        double output = (error * Kp) + (derivative * Kd)  + (integralSum * Ki) + (reference * Kf);
//        return output;
    }


