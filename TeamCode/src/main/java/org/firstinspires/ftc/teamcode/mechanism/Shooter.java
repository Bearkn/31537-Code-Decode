package org.firstinspires.ftc.teamcode.mechanism;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Shooter {
    Intake intake = new Intake();

    Vision vision = new Vision();

    private Follower follower;
    public DcMotorEx flyWheelMotor1, flyWheelMotor2, turretMotor;
//    public double distanceFromGoal;

    public boolean spinShooter = false;

    public boolean teamColor = true;

    public boolean flyWheelActivated = false;

    public boolean shooting = false;

    public double speedChange = 57000;


    public double speed = 1700;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // Start Pose of our robot.


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

    public void spin() {
        if (spinShooter) {
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

    public void speedCalc(double distance) {

        if (distance >2) {
            speed = 1800;
        } else {
            speed = 2200;
        }
//        speed = Math.sqrt(distance * speedChange);
    }

    public void spinTurret(double angle) {
        angle = MathFunctions.clamp(angle, -75, 75);
        double error = angle - angleNormalize(turretMotor.getCurrentPosition());
        if (Math.abs(error) > -.5) {
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
            Pose redGoal = new Pose(63, 64.5, Math.toDegrees(0)); // Start Pose of our robot.
            double distanceFromGoal = Math.sqrt(Math.pow(robotX - 64, 2) + Math.pow(robotY - 63.5, 2));

            double distFromGoalY = 63.5 - robotY;

            double angleToGoal = -(Math.acos(distFromGoalY / distanceFromGoal) * (180 / Math.PI));

            angleTurret = angleToGoal - (robotTheta * (180 / Math.PI));

            angleTurret = angleTurret + 90;

            speedCalc(distanceFromGoal);


        } else {
            Pose blueGoal = new Pose(-64, 63.5, Math.toDegrees(0)); // Start Pose of our robot.

            double distanceFromGoal = Math.sqrt(Math.pow(robotX - (-64), 2) + Math.pow(robotY - 63.5, 2));

            double distFromGoalY = 63.5 - robotY;

            double angleToGoal = -(Math.acos(distFromGoalY / distanceFromGoal) * (180 / Math.PI));

            angleTurret = angleToGoal - (robotTheta * (180 / Math.PI));
            angleTurret = angleTurret + 180;


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

    public void shootAuton() {

        if (Math.abs(flyWheelMotor1.getVelocity() - 1800) < 50) {
            intake.turretSpinIntake = true;
            intake.hardStopActivated = true;

        } else {
//           intake.turretSpinIntake = false;
            intake.hardStopActivated = false;
        }

    }

    public void turretAuton(){
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(vision.llResult != null && vision.llResult.isValid() && Math.abs(turretMotor.getCurrentPosition()) < 450) {
            if(Math.abs(vision.llResult.getTx()) > .5) {
                turretMotor.setPower(MathFunctions.clamp(-(vision.llResult.getTx() / 40), -.4, .4));
            } else {
                turretMotor.setPower(0);
            }
        } else {
            spinTurret(0);
        }
    }

}





