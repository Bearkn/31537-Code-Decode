package org.firstinspires.ftc.teamcode.Autons;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
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
public class Blue15Close extends OpMode{
    private Follower follower;
    MecanumDrive drive = new MecanumDrive();
    Shooter shooter = new Shooter();

    Intake intake = new Intake();

    Turret turret = new Turret();

    FileManager fileManager = new FileManager();

    private ElapsedTime pathTimer = new ElapsedTime();
    private ElapsedTime moveTimer = new ElapsedTime();



    //state variables


    private final Pose startPose = new Pose(33.000-72, 136.000-72, Math.toRadians(270)); // Start Pose of our robot.


    public enum PathState {
        START_MOVE,
        DRIVE_TO_FIRST,
        SHOOT_FIRST,
        DRIVE_TO_SECOND,
        DRIVE_TO_SHOOT,
        SHOOT_SECOND,
        DRIVE_TO_LEVER,
        GET_LEVER,
        WAIT_LEVER,
        DRIVE_TO_SHOOT_3,
        SHOOT_THIRD,
        DRIVE_TO_THIRD,
        DRIVE_TO_THIRD_TWO,
        DRIVE_TO_SHOOT_4,
        SHOOT_FOURTH,
        DRIVE_TO_FORTH,
        DRIVE_TO_SHOOT_5,
        DRIVE_TO_SHOOT_5_1,
        SHOOT_FIFTH,
        DRIVE_TO_END


    }
    PathState Pathstate;
    public PathChain MOVETOSHOOT,GRABFIRST,MOVETOSHOOT2,GRABSECOND,LEVER,MOVETOSHOOT3,GRABTHIRD1,GRABTHIRD2,MOVETOSHOOT4,GRABFOURTH,MOVETOSHOOT5,SETUP;

    public void buildPaths() {
        MOVETOSHOOT = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(33.000-72, 136.000-72),

                                new Pose(52.000-72, 92.000-72)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))

                .build();

        GRABFIRST = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.000-72, 92.000-72),
                                new Pose(54.000-72, 54.000-72),
                                new Pose(15.000-72, 60.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        MOVETOSHOOT2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000-72, 60.000-72),
                                new Pose(48.000-72, 60.000-72),
                                new Pose(52.000-72, 92.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(215))

                .build();

        GRABSECOND = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.000-72, 92.000-72),
                                new Pose(45.000-72, 84.000-72),
                                new Pose(20.000-72, 78.000-72)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(215),Math.toRadians(180))

                .build();

        LEVER = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(20.000-72, 78.000-72),
                                new Pose(30.000-72, 72.000-72),
                                new Pose(20.000-72, 72.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        MOVETOSHOOT3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(20.000-72, 72.000-72),

                                new Pose(52.000-72, 92.000-72)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        GRABTHIRD1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.000-72, 92.000-72),
                                new Pose(46.000-72, 36.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        GRABTHIRD2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.000-72, 92.000-72),
                                new Pose(57-72,24-72),
                                new Pose(20.000-72, 36.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        MOVETOSHOOT4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(20.000-72, 36.000-72),

                                new Pose(52.000-72, 92.000-72)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .setBrakingStart(.7)
                .build();

        GRABFOURTH = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.000-72, 92.000-72),
                                new Pose(6.000-72, 45.000-72),
                                new Pose(9.000-72, 17.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(270))

                .build();

        MOVETOSHOOT5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.000-72, 17.000-72),

                                new Pose(52.000-72, 92.000-72)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        SETUP = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(52.000-72, 92.000-72),

                                new Pose(35.000-72, 78.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .setTValueConstraint(.97)

                .build();

    }

    public void autonomousPathUpdate() {
        switch (Pathstate) {

            case START_MOVE:
                intake.intakeOn = true;
                shooter.shooterActivated = true;
                if(!follower.isBusy()) {
                    follower.followPath(MOVETOSHOOT,true);
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

                    follower.followPath(GRABFIRST);    // start next movement
                    setPathState(PathState.DRIVE_TO_SECOND);
                }
                break;

            // 3️⃣ Drive to second point
            case DRIVE_TO_SECOND:
                if (!follower.isBusy() && follower.getPose().getX() < -45) {
                    follower.followPath(MOVETOSHOOT2,.8,true);
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

                    follower.followPath(GRABSECOND);    // start next movement
                    setPathState(PathState.DRIVE_TO_LEVER);
                }
                break;
            case DRIVE_TO_LEVER:
                if (!follower.isBusy()) {
                    follower.followPath(LEVER,true);
                    setPathState(PathState.GET_LEVER);
                }
                break;
            case GET_LEVER:
                if (follower.atParametricEnd()) {
                    setPathState(PathState.WAIT_LEVER);
                }
                break;
            case WAIT_LEVER:
                if (pathTimer.seconds() > 1.25) {

                    follower.followPath(MOVETOSHOOT3,.8,true);    // start next movement
                    setPathState(PathState.DRIVE_TO_SHOOT_3);
                }
                break;
            case DRIVE_TO_SHOOT_3:
                if (follower.atParametricEnd()) {
                    setPathState(PathState.SHOOT_THIRD);
                }
                break;
            case SHOOT_THIRD:
                if(!follower.isBusy()){
                    follower.breakFollowing();
                }
                if (pathTimer.seconds() < 1.25) {
                    intake.stopOn = false;
                } else {

                    intake.stopOn = true;

                    follower.followPath(GRABTHIRD2,true);    // start next movement
                    setPathState(PathState.DRIVE_TO_THIRD_TWO);
                }
                break;
//            case DRIVE_TO_THIRD:
//                if (!follower.isBusy()) {
//                    follower.followPath(GRABTHIRD2);
//                    setPathState(PathState.DRIVE_TO_THIRD_TWO);
//                }
//                break;
            case DRIVE_TO_THIRD_TWO:
                if (!follower.isBusy()) {
                    follower.followPath(MOVETOSHOOT4,.8,true);
                    setPathState(PathState.DRIVE_TO_SHOOT_4);
                }
                break;
            case DRIVE_TO_SHOOT_4:
                if (follower.atParametricEnd()) {
                    setPathState(PathState.SHOOT_FOURTH);
                }
                break;
            case SHOOT_FOURTH:
                if (pathTimer.seconds() < 1.25) {
                    intake.stopOn = false;
                } else {

                    intake.stopOn = true;

                    follower.followPath(GRABFOURTH);    // start next movement
                    setPathState(PathState.DRIVE_TO_FORTH);
                }
                break;
            case DRIVE_TO_FORTH:
                if (!follower.isBusy()) {
                    follower.followPath(MOVETOSHOOT5,.8,true);
                    setPathState(PathState.DRIVE_TO_SHOOT_5);
                }
                break;
            case DRIVE_TO_SHOOT_5:
                if (follower.getCurrentTValue() < .3 && follower.getCurrentTValue() >= .2) {
                    intake.intakeOn = false;
                } else if (follower.getCurrentTValue() >=.4) {
                    intake.intakeOn = true;
                    setPathState(PathState.DRIVE_TO_SHOOT_5_1);
                }
                break;

            case DRIVE_TO_SHOOT_5_1:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_FIFTH);
                }
                break;
            case SHOOT_FIFTH:
                if (pathTimer.seconds() < 1.25) {
                    intake.stopOn = false;
                } else {

                    intake.stopOn = true;

                    follower.followPath(SETUP,.6,true);    // start next movement
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