package org.firstinspires.ftc.teamcode.Autons;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Algs.FileManager;
import org.firstinspires.ftc.teamcode.mechanism.Intake;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Shooter;
import org.firstinspires.ftc.teamcode.mechanism.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class Red21Close extends OpMode{
    private Follower follower;
    MecanumDrive drive = new MecanumDrive();
    Shooter shooter = new Shooter();

    Intake intake = new Intake();

    Turret turret = new Turret();

    FileManager fileManager = new FileManager();

    private ElapsedTime pathTimer = new ElapsedTime();
    private ElapsedTime moveTimer = new ElapsedTime();



    //state variables


    private final Pose startPose = new Pose(111-72, 136-72, Math.toRadians(270)); // Start Pose of our robot.


    public enum PathState {
        START_MOVE,
        DRIVE_TO_FIRST,
        SHOOT_FIRST,
        DRIVE_TO_SECOND,
        DRIVE_TO_SHOOT,
        SHOOT_SECOND,
        DRIVE_TO_THIRD,
        DRIVE_TO_SHOOT_2,
        SHOOT_THIRD,
        CLEAR1,
        WAIT_LEVER1,
        DRIVE_TO_SHOOT_3,
        SHOOT_FOURTH,
        CLEAR2,
        WAIT_LEVER2,
        DRIVE_TO_SHOOT_4,
        SHOOT_FIFTH,
        CLEAR3,
        WAIT_LEVER3,
        DRIVE_TO_SHOOT_5,
        SHOOT_SIXTH,
        DRIVE_TO_END



    }
    PathState Pathstate;
    public PathChain MOVETOSHOOT1,GRAB1,MOVETOSHOOT2,GRAB2,MOVETOSHOOT3,GRAB3,LEVER1,GOTOSHOOT4,GRAB4,LEVER2,GOTOSHOOT5,GRAB5,LEVER3,GOTOSHOOT6,GRAB6,GOTOSHOOT7;

    public void buildPaths() {
        MOVETOSHOOT1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(111.000-72, 136.000-72),

                                new Pose(92.000-72, 92.000-72)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-35))

                .build();

        GRAB1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(92.000-72, 92.000-72),

                                new Pose(114.000-72, 90.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        MOVETOSHOOT2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(114.000-72, 90.000-72),

                                new Pose(92.000-72, 92.000-72)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        GRAB2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(92.000-72, 92.000-72),
                                new Pose(93.000-72, 60.000-72),
                                new Pose(118.000-72, 63.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        MOVETOSHOOT3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(118.000-72, 63.000-72),

                                new Pose(92.000-72, 92.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        GRAB3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(92.000-72, 92.000-72),
                                new Pose(111.000-72, 66.000-72),
                                new Pose(133.000-72, 61.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(25))

                .build();

        LEVER1 = follower.pathBuilder().addPath(
                        new BezierPoint(
                                new Pose(133-72, 61.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(25))

                .build();

        GOTOSHOOT4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(133.000-72, 61.000-72),

                                new Pose(92.000-72, 92.000-72)
                        )
                )
//                .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(-45))
                .setConstantHeadingInterpolation(Math.toRadians(-45))
                .build();

        GRAB4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(92.000-72, 92.000-72),
                                new Pose(111.000-72, 66.000-72),
                                new Pose(133.000-72, 61.000-72)
                        )
                )
//                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(25))
                .setConstantHeadingInterpolation(Math.toRadians(25))
                .build();

        LEVER2 = follower.pathBuilder().addPath(
                        new BezierPoint(
                                new Pose(133-72, 61.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(25))

                .build();

        GOTOSHOOT5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(133.000-72, 61.000-72),

                                new Pose(92.000-72, 92.000-72)
                        )
                )
//                .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(-45))
                .setConstantHeadingInterpolation(Math.toRadians(-45))

                .build();

        GRAB5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(92.000-72, 92.000-72),
                                new Pose(111.000-72, 66.000-72),
                                new Pose(133.000-72, 61.000-72)
                        )
                )
//                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(25))
                .setConstantHeadingInterpolation(Math.toRadians(25))
                .build();

        LEVER3 = follower.pathBuilder().addPath(
                        new BezierPoint(
                                new Pose(133-72, 61.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(25))

                .build();

        GOTOSHOOT6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(133.000-72, 61.000-72),

                                new Pose(92.000-72, 92.000-72)
                        )
                )
//                .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(-45))
                .setConstantHeadingInterpolation(Math.toRadians(-45))
                .build();

        GRAB6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(92.000-72, 92.000-72),
                                new Pose(81.000-72, 30.000-72),
                                new Pose(129.000-72, 33.000-72)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))

                .build();

        GOTOSHOOT7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(129.000-72, 33.000-72),

                                new Pose(87.000-72, 108.000-72)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-60))

                .build();

    }

    public void autonomousPathUpdate() {
        switch (Pathstate) {

            case START_MOVE:
                intake.intakeOn = true;
                shooter.shooterActivated = true;
                if(!follower.isBusy()) {
                    follower.followPath(MOVETOSHOOT1,true);
                    setPathState(PathState.SHOOT_FIRST);
                }
                break;
            // 1️⃣ Drive to first point
            case DRIVE_TO_FIRST:
                if (follower.atParametricEnd()) {
                    setPathState(PathState.SHOOT_FIRST);
                }
                break;

            // 2️⃣ Shoot for 1 second
            case SHOOT_FIRST:
                if (pathTimer.seconds() < 3) {
                    intake.stopOn = false;
                } else {

                    intake.stopOn = true;

                    follower.followPath(GRAB1);    // start next movement
                    setPathState(PathState.DRIVE_TO_SECOND);
                }
                break;

            // 3️⃣ Drive to second point
            case DRIVE_TO_SECOND:
                if (!follower.isBusy()) {
                    follower.followPath(MOVETOSHOOT2,true);
                    setPathState(PathState.DRIVE_TO_SHOOT);
                }
                break;
            case DRIVE_TO_SHOOT:
                if (follower.atParametricEnd()) {
                    setPathState(PathState.SHOOT_SECOND);
                }
                break;
            case SHOOT_SECOND:
                if (pathTimer.seconds() < 1.25) {
                    intake.stopOn = false;
                } else {

                    intake.stopOn = true;

                    follower.followPath(GRAB2);    // start next movement
                    setPathState(PathState.DRIVE_TO_THIRD);
                }
                break;
            case DRIVE_TO_THIRD:
                if (!follower.isBusy()) {
                    follower.followPath(MOVETOSHOOT3,true);
                    setPathState(PathState.DRIVE_TO_SHOOT_2);
                }
                break;
            case DRIVE_TO_SHOOT_2:
                if (follower.atParametricEnd()) {
                    setPathState(PathState.SHOOT_THIRD);
                }
                break;
            case SHOOT_THIRD:
                if (pathTimer.seconds() < 1.25) {
                    intake.stopOn = false;
                } else {

                    intake.stopOn = true;

                    follower.followPath(GRAB3,true);    // start next movement
                    setPathState(PathState.CLEAR1);
                }
                break;
            case CLEAR1:
                if(!follower.isBusy()){
//                    follower.followPath(LEVER1);
                    setPathState(PathState.WAIT_LEVER1);
                }
                break;
            case WAIT_LEVER1:
                if (pathTimer.seconds() > 1.25) {
                    follower.breakFollowing();
                    follower.followPath(GOTOSHOOT4,true);    // start next movement
                    setPathState(PathState.DRIVE_TO_SHOOT_3);
                }
                break;
            case DRIVE_TO_SHOOT_3:
                if (follower.atParametricEnd()) {
                    setPathState(PathState.SHOOT_FOURTH);
                }
                break;
            case SHOOT_FOURTH:
                if (pathTimer.seconds() < 1.25) {
                    intake.stopOn = false;
                } else {

                    intake.stopOn = true;

                    follower.followPath(GRAB4,true);    // start next movement
                    setPathState(PathState.CLEAR2);
                }
                break;
            case CLEAR2:
                if(!follower.isBusy()){
//                    follower.followPath(LEVER2);
                    setPathState(PathState.WAIT_LEVER2);
                }
                break;
            case WAIT_LEVER2:
                if (pathTimer.seconds() > 1.25) {
                    follower.breakFollowing();
                    follower.followPath(GOTOSHOOT5,true);    // start next movement
                    setPathState(PathState.DRIVE_TO_SHOOT_4);
                }
                break;
            case DRIVE_TO_SHOOT_4:
                if (follower.atParametricEnd()) {
                    setPathState(PathState.SHOOT_FIFTH);
                }
                break;
            case SHOOT_FIFTH:
                if (pathTimer.seconds() < 1.25) {
                    intake.stopOn = false;
                } else {

                    intake.stopOn = true;

                    follower.followPath(GRAB5,true);    // start next movement
                    setPathState(PathState.CLEAR3);
                }
                break;
            case CLEAR3:
                if(!follower.isBusy()){
//                    follower.followPath(LEVER3);
                    setPathState(PathState.WAIT_LEVER3);
                }
                break;
            case WAIT_LEVER3:
                if (pathTimer.seconds() > 1.25) {
                    follower.breakFollowing();
                    follower.followPath(GOTOSHOOT6,true);    // start next movement
                    setPathState(PathState.DRIVE_TO_SHOOT_5);
                }
                break;
            case DRIVE_TO_SHOOT_5:
                if (follower.atParametricEnd()) {
                    setPathState(PathState.SHOOT_SIXTH);
                }
                break;
            case SHOOT_SIXTH:
                if (pathTimer.seconds() < 1.25) {
                    intake.stopOn = false;
                } else {

                    intake.stopOn = true;

//                    follower.followPath(GRAB3,true);    // start next movement
                    setPathState(PathState.DRIVE_TO_END);
                }
                break;
//            case DRIVE_TO_FOURTH:
//                if (follower.atParametricEnd()) {
//                    setPathState(PathState.WAIT_LEVER);
//                }
//                break;
//            case WAIT_LEVER:
//                if (pathTimer.seconds() > 1.25) {
//
//                    follower.followPath(MOVETOSHOOT4,true);    // start next movement
//                    setPathState(Blue15Close.PathState.DRIVE_TO_SHOOT_3);
//                }
//                break;
//            case DRIVE_TO_FORTH:
//                if (!follower.isBusy()) {
//                    follower.followPath(MOVETOSHOOT4,true);
//                    setPathState(PathState.DRIVE_TO_SHOOT_3);
//                }
//                break;
//            case DRIVE_TO_SHOOT_3:
//                if (follower.atParametricEnd()) {
//                    setPathState(PathState.SHOOT_FOURTH);
//                }
//                break;
//            case SHOOT_FOURTH:
//                if(!follower.isBusy()){
//                    follower.breakFollowing();
//                }
//                if (pathTimer.seconds() < 1.25) {
//                    intake.stopOn = false;
//                } else {
//
//                    intake.stopOn = true;
//
//                    follower.followPath(GRAB3,true);    // start next movement
//                    setPathState(PathState.DRIVE_TO_END);
//                }
//                break;
//            case
//            case DRIVE_TO_THIRD:
//                if (!follower.isBusy()) {
//                    follower.followPath(GRABTHIRD2);
//                    setPathState(PathState.DRIVE_TO_THIRD_TWO);
//                }
//                break;
//            case DRIVE_TO_THIRD_TWO:
//                if (!follower.isBusy()) {
//                    follower.followPath(MOVETOSHOOT4,.8,true);
//                    setPathState(PathState.DRIVE_TO_SHOOT_4);
//                }
//                break;
//            case DRIVE_TO_SHOOT_4:
//                if (follower.atParametricEnd()) {
//                    setPathState(PathState.SHOOT_FOURTH);
//                }
//                break;
//            case SHOOT_FOURTH:
//                if (pathTimer.seconds() < 1.25) {
//                    intake.stopOn = false;
//                } else {
//
//                    intake.stopOn = true;
//
//                    follower.followPath(GRABFOURTH);    // start next movement
//                    setPathState(PathState.DRIVE_TO_FORTH);
//                }
//                break;
//            case DRIVE_TO_FORTH:
//                if (!follower.isBusy()) {
//                    follower.followPath(MOVETOSHOOT5,.8,true);
//                    setPathState(PathState.DRIVE_TO_SHOOT_5);
//                }
//                break;
//            case DRIVE_TO_SHOOT_5:
//                if (follower.getCurrentTValue() < .3 && follower.getCurrentTValue() >= .2) {
//                    intake.intakeOn = false;
//                } else if (follower.getCurrentTValue() >=.4) {
//                    intake.intakeOn = true;
//                    setPathState(PathState.DRIVE_TO_SHOOT_5_1);
//                }
//                break;
//
//            case DRIVE_TO_SHOOT_5_1:
//                if (!follower.isBusy()) {
//                    setPathState(PathState.SHOOT_FIFTH);
//                }
//                break;
//            case SHOOT_FIFTH:
//                if (pathTimer.seconds() < 1.25) {
//                    intake.stopOn = false;
//                } else {
//
//                    intake.stopOn = true;
//
//                    follower.followPath(SETUP,.6,true);    // start next movement
//                    setPathState(PathState.DRIVE_TO_END);
//                }
//                break;
            case DRIVE_TO_END:
                if(!follower.isBusy()){
//                    follower.breakFollowing();
                }
                break;
        }
    }


    public void setPathState(PathState pState) {
        Pathstate = pState;
        pathTimer.reset();
    }

    @Override
    public void loop() {
        if(!intake.Outtake) {
            if (intake.intakeOn) {
                intake.intakeState = Intake.IntakeState.INTAKE;
                intake.indexState = Intake.IndexState.INTAKE;

            } else {
                intake.intakeState = Intake.IntakeState.STOP;
                intake.indexState = Intake.IndexState.STOP;
            }
        } else {
            intake.intakeState = Intake.IntakeState.OUTTAKE;
            intake.indexState = Intake.IndexState.OUTTAKE;
        }

        if(!intake.stopOn){
            if(!intake.intakeOn){
                intake.indexState = Intake.IndexState.INTAKE;
                intake.intakeState = Intake.IntakeState.SHOOT;
            }
            if(shooter.currentFlywheelSpeed >= Math.abs(shooter.targetFlywheelSpeed-50)) {
                intake.stopState = Intake.StopState.SHOOT;
            }
        } else {
            intake.stopState = Intake.StopState.HOLD;
        }

        follower.update();
        autonomousPathUpdate();
        turret.update(turret.turretpositionX(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),turret.turretpositionY(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),Math.toDegrees(follower.getHeading()),turret.redGoalX,turret.redGoalY,follower.getVelocity().getXComponent(),follower.getVelocity().getYComponent(),false,false);
        shooter.update(Shooter.distance2D(turret.turretpositionX(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),turret.turretpositionY(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()), turret.redGoalX,turret.redGoalY),shooter.currentFlywheelSpeed);
        intake.update();
        follower.update();
        turret.FFturret(follower.getHeading());



        telemetry.addData("path state", Pathstate.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("shooter rpm",shooter.currentFlywheelSpeed);
        telemetry.addData("pathTimer", pathTimer.seconds());

        telemetry.addData("current velo", shooter.currentFlywheelSpeed);

        telemetry.update();



    }
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        setPathState(PathState.START_MOVE);
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        turret.init(hardwareMap);
        fileManager.init();
        pathTimer = new ElapsedTime();
        pathTimer.reset();
        moveTimer = new ElapsedTime();
        moveTimer.reset();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);


        //state variables
        intake.intakeOn = false;
        intake.Outtake = false;
        intake.stopOn = true;


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
        setPathState(PathState.START_MOVE);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        fileManager.FileWrite(follower.getPose().getX(),follower.getPose().getY(),follower.getHeading());
    }



}