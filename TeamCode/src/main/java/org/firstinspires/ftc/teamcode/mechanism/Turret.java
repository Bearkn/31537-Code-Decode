package org.firstinspires.ftc.teamcode.mechanism;

import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

public class Turret {

    //turret obj

    public Servo turretServoBack;
    public Servo turretServoFront;


    //turret variables

    private double maxAngle = 150.1;

    public double turretAngle = 0.0f;

    public boolean shooterActivated = true;

    public void init(HardwareMap hwMap){
        turretServoBack = hwMap.get(Servo.class,"btservo");
        turretServoFront = hwMap.get(Servo.class,"ftservo");
    }
    public double angleToUnit(double angle) {
        return (angle + 180) / 360;
    }

    private double setAngleToTurret() {
        // Clamp angle
        double clampedAngle = MathFunctions.clamp(turretAngle,-maxAngle,maxAngle);

        // Convert from [-180, 180] to [0, 1]
        return angleToUnit(clampedAngle);
    }


    public void update(){
        if(shooterActivated) {
            turretServoFront.setPosition(setAngleToTurret());
            turretServoBack.setPosition(setAngleToTurret());
        }
    }
}
