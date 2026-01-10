package org.firstinspires.ftc.teamcode.mechanism;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.AnalogInput;
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

    // Servos
//    public CRServoGroup turretServos;
    public Servo turretServoFront;
    public Servo turretServoBack;


//    public AbsoluteAnalogEncoder turretEncoder;




    // Turret limits
    private static final double MAX_ANGLE = 150.0;

    public double turretAngle = 0.0;
    public boolean shooterActivated = true;

    public double fieldAngle;

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



    PIDF turretpidf = new PIDF(
            0.003,  // kP
            0.0,    // kI
            0.0002, // kD
            0.0     // kF (usually 0 for CR)
    );


    private double angleToServo(double angle360) {
        // Map 0–360° → 0–1
        return angle360 / 360.0;
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

        double angle = fieldAngle - robotHeading;

        if (angle < 0) angle += 360;                        // [0,360)

        return angleToServo(angle);
    }

    private double getRawAngle(AnalogInput analog) {
        return (analog.getVoltage() / ANALOG_MAX_VOLTAGE) * SERVO_RANGE_DEG;
    }



    private boolean atTarget(double target, double current, double tolerance) {
        return Math.abs(target - current) <= tolerance;
    }


    private double clampTurretTarget(double target) {
        return Math.max(0.125, Math.min(.875, target));
    }

    public double turretpositionX(double robotX, double robotY, double robotHeading) {
        double offset = 1.25; // inches from back of robot
        return robotX - offset * Math.cos(robotHeading);
    }

    public double turretpositionY(double robotX, double robotY, double robotHeading) {
        double offset = 1.25; // inches from back of robot
        return robotY - offset * Math.sin(robotHeading);
    }



    public void update(
            double robotX,
            double robotY,
            double robotHeading,
            double goalX,
            double goalY
    ) {
        turretAngle = calculateTurretAngle(robotX, robotY, robotHeading, goalX, goalY);


        if (shooterActivated) {
            turretServoFront.setPosition(clampTurretTarget(turretAngle));
            turretServoBack.setPosition(clampTurretTarget(turretAngle));
//            turretServos.set(.1);
        }
    }
}
