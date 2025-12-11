package org.firstinspires.ftc.teamcode.Autons;


import com.pedropathing.follower.Follower;
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

@Autonomous(name = "RedSide", group = "Autons")

public class RedAuton extends OpMode{
    private Follower follower;

    Shooter shooter = new Shooter();

    Intake intake = new Intake();

    Vision vision = new Vision();
    private ElapsedTime pathTimer = new ElapsedTime();

    boolean shootBall = false;

    boolean activateHardstop = false;


//    private int pathState = 0;



    private final Pose startPose = new Pose(39, 60.5, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(20, 15.475, Math.toRadians(53)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.


    public enum PathState {
        FIRST,
        SHOOT1,
        INTAKEFIRSTROW,
        GOTOSHOOT,
        SHOOT2,
        SETUPSECONDROW,
        INTAKESECONDROW,

        DONOTGETHIT,
        GOTOSHOOT2,
        SHOOT3,
        SETUPTHIRDROW,
        INTAKETHIRDROW,
        SETUPGATE
    }
    PathState Pathstate;
    public PathChain path1,path2,path3,path4,path5,path6,path7,path8,path9,donothit;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//        move = new Path(new BezierLine(startPose, scorePose));
//        move.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(39, 60.5),new Pose(20, 15.475)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(53))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(20, 15.475),new Pose(60, 12)))
                .setLinearHeadingInterpolation(Math.toRadians(53), Math.toRadians(0))
                .setTangentHeadingInterpolation()
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(54,12),scorePose.getPose()))
                .setLinearHeadingInterpolation(Math.toRadians(0), scorePose.getHeading())
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose.getPose(),new Pose(20, -12)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(0))
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(20,-12),new Pose(72, -12)))
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(0))
                .build();


        path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(54,-12),scorePose.getPose()))
                .setLinearHeadingInterpolation(Math.toRadians(0), scorePose.getHeading())
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose.getPose(),new Pose(20, -36)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(0))
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(20,-36),new Pose(70, -36)))
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(0))
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56,-36),new Pose(36, 0)))
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(0))
                .build();



    }

    public void autonomousPathUpdate() {
        switch (Pathstate) {
            case FIRST:
                // When the path is done\
                intake.speed = .7;
                intake.spinIntake = true;
                shooter.spinShooter = true;
                if (!follower.isBusy()) {
                    follower.followPath(path1);
                    setPathState(PathState.SHOOT1);
                }

                break;


            case SHOOT1:
                if (pathTimer.seconds() < 5 && pathTimer.seconds() > 2.5) {

                    shootBall = true;
                } else if (pathTimer.seconds() >= 5) {
                    shooter.spinShooter = false;
                    shootBall = false;
                    setPathState(PathState.INTAKEFIRSTROW);
                }
                break;

            case INTAKEFIRSTROW:

                intake.speed = .8;
                intake.spinIntake = true;
                shooter.spinShooter = true;
                if (!follower.isBusy()) {
                    follower.followPath(path2,.5,true);
                    setPathState(PathState.GOTOSHOOT);
                }
                break;
            case GOTOSHOOT:
                if(follower.getPose().getX() > 54){
                    follower.followPath(path3);
                    setPathState(PathState.SHOOT2);
                }
                break;
            case SHOOT2:
                if (pathTimer.seconds() < 5 && pathTimer.seconds() > 2.5) {

                    shootBall = true;
                } else if (pathTimer.seconds() >= 5) {
                    shooter.spinShooter = false;
                    shootBall = false;
                    setPathState(PathState.SETUPSECONDROW);
                }
                break;
            case SETUPSECONDROW:
                intake.speed = .9;
                intake.spinIntake = true;
                if (!follower.isBusy()) {
                    follower.followPath(path4,true);
                    setPathState(PathState.INTAKESECONDROW);
                }
                break;
            case INTAKESECONDROW:
                if (!follower.isBusy()) {
                    follower.followPath(path5,true);
                    setPathState(PathState.GOTOSHOOT2);
                    shooter.spinShooter = true;
                }
                break;

//            case DONOTGETHIT   :
//                if (follower.getPose().getX() > 58) {
//                    follower.followPath(donothit,true);
//                    setPathState(PathState.GOTOSHOOT2);
//                    shooter.spinShooter = true;
//                }
//                break;
            case GOTOSHOOT2:
                if(follower.getPose().getX() > 54){
                    follower.followPath(path6);
                    setPathState(PathState.SHOOT3);
                }
                break;
            case SHOOT3:
                if (pathTimer.seconds() < 5 && pathTimer.seconds() > 2.5) {

                    shootBall = true;
                } else if (pathTimer.seconds() >= 5) {
                    shooter.spinShooter = false;
                    shootBall = false;
                    setPathState(PathState.SETUPTHIRDROW);
                }
                break;
            case SETUPTHIRDROW:
                intake.speed = .8;
                intake.spinIntake = true;
                if (!follower.isBusy()) {
                    follower.followPath(path7,true);
                    setPathState(PathState.INTAKETHIRDROW);
                }
                break;
            case INTAKETHIRDROW:
                if (!follower.isBusy()) {
                    follower.followPath(path8,.5,true);
                    setPathState(PathState.SETUPGATE);
                }
                break;
            case SETUPGATE:
                if (follower.getPose().getX() > 56) {
                    follower.followPath(path9);
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

            if (Math.abs(shooter.flyWheelMotor1.getVelocity() - shooter.speed) < 30) {
                intake.turretSpinIntake = true;
                activateHardstop = true;

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
            shooter.speedCalc(vision.llResult.getTy());

            telemetry.update();


        }
    }
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        shooter.speed = 1650;
        setPathState(PathState.FIRST);
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
        vision.limelight.pipelineSwitch(0); //april tag 24
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
        setPathState(PathState.FIRST);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}
