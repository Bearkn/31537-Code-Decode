package org.firstinspires.ftc.teamcode.Autons;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanism.Intake;
import org.firstinspires.ftc.teamcode.mechanism.Shooter;
import org.firstinspires.ftc.teamcode.mechanism.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "blueSide12", group = "Autons")

public class BLUEAUTON12BALL extends OpMode{
    private Follower follower;

    Shooter shooter = new Shooter();

    Intake intake = new Intake();

    Vision vision = new Vision();
    private ElapsedTime pathTimer = new ElapsedTime();

    boolean shootBall = false;

    boolean activateHardstop = false;


//    private int pathState = 0;



    private final Pose startPose = new Pose(-39, 60.5, Math.toRadians(90)); // Start Pose of our robot.


    public enum PathState {
        ONE,
        TWO,
        THREE,
        FOUR,
        FIVE,
        SIX,
        SEVEN,
        EIGHT,
        NINE,
        TEN,
        ELEVEN,
        TWELVE,
        THIRTEEN,
        FOURTEEN,
        FIFTEEN
    }
    PathState Pathstate;
    public PathChain path1,path2,path3,path4,path5,path6,path7,path8,path9;

    public void buildPaths() {
        path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(-39, 60.5), new Pose(-20, 15))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(127))
                .build();

        path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(-20, 15), new Pose(-60, 12))
                )
                .setTangentHeadingInterpolation()
                .build();

        path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(-54, 12), new Pose(-56, 4))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();

        path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(-56, 4), new Pose(-20, 15))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(127))
                .build();

        path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(-20, 15),
                                new Pose(-10, -17),
                                new Pose(-64, -14)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(127), Math.toRadians(180))
                .build();

        path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(-58, -14),
                                new Pose(-21, -5),
                                new Pose(-20, 15)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(127))
                .build();

        path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(-20, 15),
                                new Pose(-3, -51),
                                new Pose(-64, -36)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(127), Math.toRadians(180))
                .build();

        path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(-58, -36.000), new Pose(-20, 15))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(127))
                .build();

        path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(-20, 15), new Pose(-40, 0))
                )
                .setLinearHeadingInterpolation(Math.toRadians(127), Math.toRadians(180))
                .build();

    }

    public void autonomousPathUpdate() {
        switch (Pathstate) {
            case ONE:
                // When the path is done\
                intake.speed = .7;
                intake.spinIntake = true;
                shooter.spinShooter = true;
                if (!follower.isBusy()) {
                    follower.followPath(path1);
                    setPathState(PathState.TWO);
                }

                break;


            case TWO:
                if (pathTimer.seconds() < 4 && pathTimer.seconds() > 1.5) {

                    shootBall = true;
                } else if (pathTimer.seconds() >= 5) {
                    shooter.spinShooter = false;
                    shootBall = false;
                    setPathState(PathState.THREE);
                }
                break;

            case THREE:

                intake.speed = .8;
                intake.spinIntake = true;
                shooter.spinShooter = true;
                if (!follower.isBusy()) {
                    follower.followPath(path2,.5,true);
                    setPathState(PathState.FOUR);
                }
                break;
            case FOUR:
                if(follower.getPose().getX() < -53){
                    follower.followPath(path3,.7,true);
                    setPathState(PathState.FIVE);
                }
                break;
            case FIVE:
                if(pathTimer.seconds() >=2) {
                    setPathState(PathState.SIX);
                }
                break;
            case SIX:
                if(!follower.isBusy()){
                    follower.followPath(path4);
                    setPathState(PathState.SEVEN);
                }
                break;
            case SEVEN:
                if (pathTimer.seconds() < 4 && pathTimer.seconds() > 2.5) {

                    shootBall = true;
                } else if (pathTimer.seconds() >= 5) {
                    shooter.spinShooter = false;
                    shootBall = false;
                    setPathState(PathState.EIGHT);
                }
                break;
            case EIGHT:
                intake.speed = .9;
                intake.spinIntake = true;
                if (!follower.isBusy()) {
                    follower.followPath(path5,true);
                    setPathState(PathState.NINE);
                }
                break;
            case NINE:
                if (follower.getPose().getX() < -58) {
                    follower.followPath(path6,true);
                    setPathState(PathState.TEN);
                    shooter.spinShooter = true;
                }
                break;
            case TEN:
                if (pathTimer.seconds() < 4 && pathTimer.seconds() > 2.5) {

                    shootBall = true;
                } else if (pathTimer.seconds() >= 5) {
                    shooter.spinShooter = false;
                    shootBall = false;
                    setPathState(PathState.ELEVEN);
                }
                break;
            case ELEVEN:
                intake.speed = .8;
                intake.spinIntake = true;
                shooter.spinShooter = true;
                if (!follower.isBusy()) {
                    follower.followPath(path7,true);
                    setPathState(PathState.TWELVE);
                }
                break;
            case TWELVE:
                if (follower.getPose().getX() < -58) {
                    follower.followPath(path8,true);
                    setPathState(PathState.THIRTEEN);
                }
                break;
            case THIRTEEN:
                if (pathTimer.seconds() < 4 && pathTimer.seconds() > 2.5) {

                    shootBall = true;
                } else if (pathTimer.seconds() >= 5) {
                    shooter.spinShooter = false;
                    shootBall = false;
                    setPathState(PathState.FOURTEEN);
                }
                break;
            case FOURTEEN:
                if (!follower.isBusy()) {
                    follower.followPath(path9,true);
                }
                break;
        }
    }
//    public void autonomousPathUpdate() {
//            switch (pathState) {
//                case :
//                    follower.followPath(path1);
//
//                    if(!follower.isBusy()) {
////                        intake.hardStopActivated = false;
////                        shooter.spinShooter = true;
//                        pathState = 1;
//                        break;
//
//                    }



//
//                case 1:
//                    intake.speed = .7;
//                    intake.spinIntake = true;
//                    shooter.spinShooter = true;
//                    shooter.turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//                    if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 3) {
//
//
    ////                            shooter.turretAuton();
//
//
//                            if(vision.llResult != null && vision.llResult.isValid() && Math.abs(shooter.turretMotor.getCurrentPosition()) < 450) {
//                                if(Math.abs(vision.llResult.getTx()) > .5) {
//                                    shooter.turretMotor.setPower(MathFunctions.clamp(-(vision.llResult.getTx() / 40), -.4, .4));
//                                } else {
//                                    shooter.turretMotor.setPower(0);
//                                }
//                            } else {
//                                shooter.spinTurret(0);
//                            }
//                            shooter.shootAuton();
//
//
//                        } else {
//                        intake.hardStopActivated = false;
//                        break;
//                    }





//                case 2:
//                    if(!follower.isBusy()) {
//                        follower.followPath(move3);
//                        setPathState(3);
//                        break;
//                    }
//                case 3:
//                    if(!follower.isBusy()) {
//                        follower.followPath(move4);
//                        setPathState(0);
//                        break;
//                    }

//            }
//    }

    public void setPathState(PathState pState) {
        Pathstate = pState;
        pathTimer.reset();
    }

    @Override
    public void loop() {

//
//            intake.turretSpinIntake = true;
//            intake.hardStopActivated = true;
//
////            shoot.flyWheelActivated = !shoot.flyWheelActivated;
        shooter.turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(vision.llResult != null && vision.llResult.isValid() && Math.abs(shooter.turretMotor.getCurrentPosition()) < 450) {
            if(Math.abs(vision.llResult.getTx()) > .5) {
                shooter.turretMotor.setPower(MathFunctions.clamp(-(vision.llResult.getTx() / 40), -.25, .25));
            } else {
                shooter.turretMotor.setPower(0);
            }
        } else {
            shooter.spinTurret(0);
        }
        if(shootBall) {

            if (shooter.flyWheelMotor1.getVelocity() - shooter.speed > -25 && shooter.flyWheelMotor1.getVelocity() - shooter.speed < 75) {
                intake.turretSpinIntake = true;
                activateHardstop = true;
                intake.speed = 1;

            } else {
//                intake.turretSpinIntake = false;
                activateHardstop = false;
            }
        } else {
            intake.turretSpinIntake = false;
            activateHardstop = false;
        }
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        intake.hardStopPos(activateHardstop);
        intake.spin();        // Feedback to Driver Hub for debugging
        shooter.spin();
        telemetry.addData("path state", Pathstate.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("shooter rpm",shooter.flyWheelMotor1.getVelocity());
        telemetry.addData("pathTimer", pathTimer.seconds());
        vision.loop();
        if(vision.llResult != null && vision.llResult.isValid()) {
//            Pose3D botPoseMt2 = llResult.getBotpose_MT2();
            telemetry.addData("tx", vision.llResult.getTx());
            telemetry.addData("ty", vision.llResult.getTy());
            telemetry.addData("ta", vision.llResult.getTa());
//            shooter.speedCalc(vision.llResult.getTy());

            telemetry.update();


        }
    }
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        shooter.speed = 1700;
        setPathState(PathState.ONE);
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        vision.init(hardwareMap);
        pathTimer = new ElapsedTime();
        pathTimer.reset();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        vision.limelight.pipelineSwitch(1); //april tag 24
        vision.limelight.start();
    }
    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}
    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
//        opmodeTimer.resetTimer();
        pathTimer.reset();
        setPathState(PathState.ONE);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}
