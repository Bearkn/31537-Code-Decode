package org.firstinspires.ftc.teamcode.mechanism;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
//import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Algs.PIDF;


public class Turret {

    // Goals
    public double blueGoalX = -66;
    public double blueGoalY = 66;
    public double redGoalX  = 66;
    public double redGoalY  = 66;

//    // Goals
//    public double SblueGoalX = -66;
//    public double SblueGoalY = 66;
//    public double SredGoalX  = 66;
//    public double SredGoalY  = 66;


    public boolean isRed = true;

    // Servos
//    public CRServoGroup turretServos;
    public Servo turretServoFront;
    public Servo turretServoBack;

    public AnalogInput turretAnalog;


//    public AbsoluteAnalogEncoder turretEncoder;




    // Turret limits
    private static final double MAX_ANGLE = 150.0;

    public double turretpos = .5;

    public double turretAngle = 0.0;

    public double analogangle = 0;
    public boolean shooterActivated = true;

    public double fieldAngle;

    public double ballFlightTime;

    public void init(HardwareMap hwMap) {
//        turretServos = new CRServoGroup(
//                new CRServoEx(hwMap, "ftservo")
//                        .setCachingTolerance(0.01)
//                        .setRunMode(CRServoEx.RunMode.RawPower),
//                new CRServoEx(hwMap, "btservo")
//                        .setCachingTolerance(0.01)
//                        .setRunMode(CRServoEx.RunMode.RawPower));
//        turretEncoder = new AbsoluteAnalogEncoder(hwMap, "banalog")
//                .zero(0)
//                .setReversed(true);
        turretServoFront = hwMap.get(Servo.class,"ftservo");
        turretServoBack = hwMap.get(Servo.class,"btservo");
        turretAnalog = hwMap.get(AnalogInput.class,"turret");



    }

    // Axon configuration
    private static final double ANALOG_MAX_VOLTAGE = 3.3;
    private static final double SERVO_RANGE_DEG = 300.0; // change if 300°

    private static final double SERVO_NEUTRAL = 0.5;
    private static final double SERVO_MIN = 0.3;
    private static final double SERVO_MAX = 0.7;

    // Encoder unwrap state
    private double lastAngle1 = 0, totalAngle1 = 0;
    private double lastAngle2 = 0, totalAngle2 = 0;




    private double angleToServo(double angle360) {
        // Map 0–360° → 0–1
        return angle360 / 360;
    }

    private double calculateTurretAngle(
            double robotX,
            double robotY,
            double robotHeading,
            double goalX,
            double goalY
    ) {
        double dx = goalX - robotX;
        double dy = goalY - robotY;

        // Field angle to target
        fieldAngle = Math.toDegrees(Math.atan2(dy, dx));

        double angle = fieldAngle - robotHeading; //210

        if (angle < 0) angle += 360;
        if (angle > 360) angle -= 360;                        // [0,360)
// [0,360)

        return angleToServo(angle);
    }

    public void calcflightTime(double distance){
        ballFlightTime = distance * .01;        // need to figure out regression line
    }

    private double getRawAngle(AnalogInput analog) {
        return (analog.getVoltage() / ANALOG_MAX_VOLTAGE) * SERVO_RANGE_DEG;
    }



    private boolean atTarget(double target, double current, double tolerance) {
        return Math.abs(target - current) <= tolerance;
    }


    private double clampTurretTarget(double target) {
        return Math.max(0.025, Math.min(.975, target));
    }

    public double turretpositionX(double robotX, double robotY, double robotHeading) {
        double offset = 1.25; // inches from back of robot
        return robotX - offset * Math.cos(robotHeading);
    }

    public double turretpositionY(double robotX, double robotY, double robotHeading) {
        double offset = 1.25; // inches from back of robot
        return robotY - offset * Math.sin(robotHeading);
    }

    public double[] shootOnTheMove(
            double turretX,
            double turretY,
            double robotVectorX,
            double robotVectorY,
            boolean isRed
    ) {
        // Select correct goal
        double goalX = isRed ? redGoalX : blueGoalX;
        double goalY = isRed ? redGoalY : blueGoalY;

        // Distance from turret to goal
        double dx = goalX - turretX;
        double dy = goalY - turretY;
        double distance = Math.hypot(dx, dy);

        // Time of flight approximation
        double time = distance / ballFlightTime;

        // Lead compensation (move goal opposite robot motion)
        double leadX = robotVectorX * time;
        double leadY = robotVectorY * time;

        double compensatedGoalX = goalX - leadX;
        double compensatedGoalY = goalY - leadY;

        return new double[]{compensatedGoalX, compensatedGoalY};
    }



    public void update(
            double robotX,
            double robotY,
            double robotHeading,
            double goalX,
            double goalY,
            double robotVectX,
            double robotVectY
    ) {
//        double[] goal = shootOnTheMove(
//                turretpositionX(robotX, robotY, robotHeading),
//                turretpositionY(robotX, robotY, robotHeading),
//                robotVectX,
//                robotVectY,
//                isRed
//        );

        turretAngle = calculateTurretAngle(robotX, robotY, robotHeading, goalX, goalY);

        turretAngle = MathFunctions.clamp(turretAngle,.025,.975);


        analogangle = (((turretAnalog.getVoltage() / 3.3)* 450)-45);

//        analogangle = MathFunctions.clamp(analogangle,.25,.75);




        if (shooterActivated) {

            turretServoFront.setPosition(clampTurretTarget(turretAngle));
            turretServoBack.setPosition(clampTurretTarget(turretAngle));
//
//            turretServoFront.setPosition(.5);
//            turretServoBack.setPosition(.5);
//            turretServos.set(.1);
        } else {
            turretServoFront.setPosition(.5);
            turretServoBack.setPosition(.5);
        }
    }
}
