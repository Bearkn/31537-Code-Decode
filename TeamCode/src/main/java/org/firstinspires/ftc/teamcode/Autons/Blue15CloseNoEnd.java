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
public class Blue15CloseNoEnd extends OpMode{
    private Follower follower;
    MecanumDrive drive = new MecanumDrive();
    Shooter shooter = new Shooter();

    Intake intake = new Intake();

    Turret turret = new Turret();

    FileManager fileManager = new FileManager();

    private ElapsedTime pathTimer = new ElapsedTime();
    private ElapsedTime moveTimer = new ElapsedTime();



    //state variables


    private final Pose startPose = new Pose(35.000-72, 135.000-72, Math.toRadians(270)); // Start Pose of our robot.


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
        GOSHOOT3,
        CLEAR1,
        WAIT_LEVER1,
        DRIVE_TO_SHOOT_3,
        SHOOT_FOURTH,
        CLEAR2,
        WAIT_LEVER2,
        DRIVE_TO_SHOOT_4,
        SHOOT_FIFTH,
        DRIVE_TO_FOURTH,
        GO_LEVER1,
        GO_LEVER2,
        SHOOT_SIXTH,
        DRIVE_TO_END,
        DRIVE_TO_FIFTH,
        DRIVE_TO_SIXTH,
        GOSHOOT4,
        ATLEVER1,
        ATLEVER2,
        INTAKESTUFF,
        INTAKESTUFF2



    }
    PathState Pathstate;
    //    public PathChain MOVETOSHOOT1,GRAB1,MOVETOSHOOT2,GRAB2,MOVETOSHOOT3,GRAB3,LEVER1,GOTOSHOOT4,GRAB4,LEVER2,GOTOSHOOT5,GRAB5,LEVER3,GOTOSHOOT6,GRAB6,GOTOSHOOT7;
    public PathChain Movetoshoot1;
    public PathChain GRAB1;
    public PathChain Movetoshoot2;
    public PathChain Gotogate1;
    public PathChain GRAB2;
    public PathChain Movetoshoot3;
    public PathChain Gotogate2;
    public PathChain GRAB3;
    public PathChain Movetoshoot4;
    public PathChain GRAB4;
    public PathChain Movetoshoot5;
    public PathChain GRAB5;
    public PathChain Path13;
    public PathChain Path14;
    public void buildPaths() {
        Movetoshoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(35.000-72, 135.000-72),

                                new Pose(52.000-72, 92.000-72)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))

                .build();

        GRAB1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.000-72, 92.000-72),
                                new Pose(54.000-72, 54.000-72),
                                new Pose(20.000-72, 60.000-72)
                        )
                )
//                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Movetoshoot2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(22.000-72, 60.000-72),
                                new Pose(48.000-72, 60.000-72),
                                new Pose(52.000-72, 92.000-72)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Gotogate1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.000-72, 92.000-72),
                                new Pose(50.000-72, 63.000-72),
                                new Pose(18.000-72, 65.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        GRAB2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(18.000-72, 65.000-72),
                                new Pose(20.000-72, 54.000-72),
                                new Pose(13.000-72, 56.000-72)
                        )
                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                .setConstantHeadingInterpolation(Math.toRadians(145))
                .build();

        Movetoshoot3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(13.000-72, 60.000-72),
                                new Pose(51.000-72, 63.000-72),
                                new Pose(52.000-72, 92.000-72)
                        )
                )
//                .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(180))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Gotogate2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.000-72, 92.000-72),
                                new Pose(50.000-72, 63.000-72),
                                new Pose(18.000-72, 65.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        GRAB3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(18.000-72, 65.000-72),
                                new Pose(20.000-72, 54.000-72),
                                new Pose(13.000-72, 56.000-72)
                        )
                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                .setConstantHeadingInterpolation(Math.toRadians(145))
                .build();

        Movetoshoot4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(13.000-72, 60.000-72),
                                new Pose(48-72,60-72),
                                new Pose(52.000-72, 92.000-72)
                        )
                )
//                .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(180))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        GRAB4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.000-72, 92.000-72),
                                new Pose(45.000-72, 80.000-72),
                                new Pose(23.000-72, 80.000-72)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Movetoshoot5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.000-72, 84.000-72),

                                new Pose(52.000-72, 92.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        GRAB5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.000-72, 92.000-72),
                                new Pose(57.000-72, 36.000-72),
                                new Pose(15.000-72, 40.000-72)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(52-72, 92-72),

                                new Pose(60.000-72, 111-72)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path14 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(52.000-72, 92.000-72),

                                new Pose(39.000-72, 81.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
    }

    public void autonomousPathUpdate() {
        switch (Pathstate) {

            case START_MOVE:
                intake.intakeOn = true;
                shooter.shooterActivated = true;
                if(!follower.isBusy()) {
                    follower.followPath(Movetoshoot1,true);
                    setPathState(PathState.DRIVE_TO_FIRST);
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
                if (pathTimer.seconds() < 1.25) {
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
                    follower.followPath(Movetoshoot2,true);
                    setPathState(PathState.DRIVE_TO_SHOOT);
                }
                break;
            case DRIVE_TO_SHOOT:
                if (follower.atParametricEnd()) {
                    setPathState(PathState.SHOOT_SECOND);
                }
                break;
            case SHOOT_SECOND:
                if (pathTimer.seconds() < 1) {
                    intake.stopOn = false;
                } else {

                    intake.stopOn = true;

                    follower.followPath(Gotogate1,.8,true);    // start next movement
                    setPathState(PathState.ATLEVER1);
                }
                break;
            case ATLEVER1:
                if (follower.atParametricEnd()) {
                    setPathState(PathState.DRIVE_TO_THIRD);
                }
                break;
            case DRIVE_TO_THIRD:
                if (pathTimer.seconds() > .5) {
                    follower.breakFollowing();
                    follower.followPath(GRAB2,true);
                    setPathState(PathState.GO_LEVER1);
                }
                break;
//            case DRIVE_TO_THIRD:
//                if (!follower.isBusy()) {
//                    follower.followPath(GRAB2,true);
//                    setPathState(PathState.GO_LEVER1);
//                }
//                break;
            case GO_LEVER1:
                if (follower.atParametricEnd()) {
                    setPathState(PathState.WAIT_LEVER1);
                }
                break;
            case WAIT_LEVER1:
                if (pathTimer.seconds() > .75) {
                    follower.breakFollowing();
                    follower.followPath(Movetoshoot3,true);    // start next movement
                    setPathState(PathState.INTAKESTUFF);
                }
                break;
            case INTAKESTUFF:
                if (follower.getCurrentTValue() < .4 && follower.getCurrentTValue() >= .3) {
                    intake.intakeOn = false;
                } else if (follower.getCurrentTValue() >=.4) {
                    intake.intakeOn = true;
                    setPathState(PathState.GOSHOOT3);
                }
                break;
            case GOSHOOT3:
                if (follower.atParametricEnd()) {
                    setPathState(PathState.SHOOT_THIRD);
                }
                break;
            case SHOOT_THIRD:
                if (pathTimer.seconds() < 1) {
                    intake.stopOn = false;
                } else {

                    intake.stopOn = true;

                    follower.followPath(Gotogate2,.8,true);    // start next movement
                    setPathState(PathState.ATLEVER2);
                }
                break;
            case ATLEVER2:
                if (follower.atParametricEnd()) {
                    setPathState(PathState.DRIVE_TO_FOURTH);
                }
                break;
            case DRIVE_TO_FOURTH:
                if (pathTimer.seconds() > 1) {
                    follower.breakFollowing();
                    follower.followPath(GRAB3,true);
                    setPathState(PathState.GO_LEVER2);
                }
                break;
            case GO_LEVER2:
                if (follower.atParametricEnd()) {
                    setPathState(PathState.WAIT_LEVER2);
                }
                break;
            case WAIT_LEVER2:
                if (pathTimer.seconds() > .75) {
                    follower.breakFollowing();
                    follower.followPath(Movetoshoot4,true);    // start next movement
                    setPathState(PathState.INTAKESTUFF2);
                }
                break;
            case INTAKESTUFF2:
                if (follower.getCurrentTValue() < .4 && follower.getCurrentTValue() >= .3) {
                    intake.intakeOn = false;
                } else if (follower.getCurrentTValue() >=.4) {
                    intake.intakeOn = true;
                    setPathState(PathState.GOSHOOT4);
                }
                break;
            case GOSHOOT4:
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
                    setPathState(PathState.DRIVE_TO_FIFTH);
                }
                break;
            case DRIVE_TO_FIFTH:
                if(!follower.isBusy()){
                    follower.followPath(Movetoshoot5);
                    setPathState(PathState.DRIVE_TO_SHOOT_3);
                }
                break;

            case DRIVE_TO_SHOOT_3:
                if (follower.atParametricEnd()) {
                    setPathState(PathState.SHOOT_FIFTH);
                }
                break;
            case SHOOT_FIFTH:
                if (pathTimer.seconds() < 1.25) {
                    intake.stopOn = false;
                } else {

                    intake.stopOn = true;

                    follower.followPath(Path13,true);    // start next movement
                    setPathState(PathState.DRIVE_TO_SIXTH);
                }
                break;
            case DRIVE_TO_SIXTH:
                if(!follower.isBusy()){
                    follower.breakFollowing();
                    setPathState(PathState.DRIVE_TO_END);
                }
                break;
            case DRIVE_TO_SHOOT_4:
                if (follower.atParametricEnd()) {
                    setPathState(PathState.SHOOT_SIXTH);
                }
                break;
            case SHOOT_SIXTH:
                if (pathTimer.seconds() < 1) {
                    intake.stopOn = false;
                } else {

                    intake.stopOn = true;

//                    follower.followPath(Path14,true);    // start next movement
                    follower.breakFollowing();
                    setPathState(PathState.DRIVE_TO_END);
                }
                break;
            case DRIVE_TO_END:
                if(!follower.isBusy()){
                    follower.breakFollowing();
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
        turret.update(turret.turretpositionX(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),turret.turretpositionY(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),Math.toDegrees(follower.getHeading()),turret.blueGoalX,turret.blueGoalY,follower.getVelocity().getXComponent(),follower.getVelocity().getYComponent(),false,false);
        shooter.update(Shooter.distance2D(turret.turretpositionX(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),turret.turretpositionY(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()), turret.blueGoalX,turret.blueGoalY),shooter.currentFlywheelSpeed);
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

        turret.offset -= .01;

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