package org.firstinspires.ftc.teamcode.Autons;


import com.pedropathing.Drivetrain;
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
public class Red18Close extends OpMode{
    private Follower follower;
    MecanumDrive drive = new MecanumDrive();
    Shooter shooter = new Shooter();

    Intake intake = new Intake();

    Turret turret = new Turret();

    FileManager fileManager = new FileManager();

    private ElapsedTime pathTimer = new ElapsedTime();
    private ElapsedTime moveTimer = new ElapsedTime();



    //state variables


    private final Pose startPose = new Pose(39, 136.000-72, Math.toRadians(270)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(20, 10, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.


    public enum PathState {
        SHOOT,
        GRABCORNER,
        SHOOT1,
        GRAB3,
        SHOOT2,
        GRABCORNER1,
        SHOOT3,
        GRABCORNER2,
        SHOOT4,
        GRABCORNER3,
        SHOOT5,
        LEAVE,
        MORE,
        LESS,
        HOME,
        ENDING
    }
    PathState Pathstate;
    public PathChain Path1,Path2,Path3,Path4,Path5,Path6,Path7,Path8,Path9,Path10,Path11,Path12,Path13,Path14;

    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(39, 136.000-72),

                                new Pose(20, 92.000-72)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(20, 92.000-72),
                                new Pose(18, 54.000-72),
                                new Pose(57, 60.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(57, 60.000-72),
                                new Pose(24, 60.000-72),
                                new Pose(20, 92.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(325))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(20, 92.000-72),
                                new Pose(24, 60.000-72),
                                new Pose(52, 66.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52, 66.000-72),
                                new Pose(54, 60.000-72),
                                new Pose(58, 58.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(35))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(58, 58-72),
                                new Pose(21, 63.000-72),
                                new Pose(20, 92.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(20, 92.000-72),
                                new Pose(27, 84.000-72),
                                new Pose(48, 80.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(48, 80.000-72),

                                new Pose(20, 92.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(20, 92.000-72),
                                new Pose(33, 69.000-72),
                                new Pose(54, 66.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(54, 66.000-72),
                                new Pose(54, 54.000-72),
                                new Pose(63, 54.000-72)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(80))

                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(63, 54.000-72),

                                new Pose(20, 92.000-72)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(80), Math.toRadians(125))

                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(20, 92.000-72),
                                new Pose(15, 30.000-72),
                                new Pose(48, 39.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Path13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(48, 36.000-72),

                                new Pose(12, 110-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(305))

                .build();

        Path14 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(20, 92.000-72),

                                new Pose(42, 81.000-72)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(125),(Math.toRadians(0)))

                .build();


    }

    public void autonomousPathUpdate() {
        switch (Pathstate) {
            case SHOOT:
                if(!follower.isBusy()) {
                    intake.intakeOn = true;
                    shooter.shooterActivated = true;
                    turret.shooterActivated = false;
                    intake.stopOn = false;
//                    if(pathTimer.seconds() >=2){
                    follower.followPath(Path1);
                    setPathState((PathState.GRABCORNER));
//                    }

                }
                break;

            case GRABCORNER:
                if (!follower.isBusy()) {
                    turret.shooterActivated = true;
                    intake.stopOn = false;
                    if(pathTimer.seconds() >= 2.9) {
                        intake.stopOn = true;
                        follower.followPath(Path2);
                        setPathState(PathState.GRAB3);
                    }
                }
                break;
            case GRAB3:
                if(!follower.isBusy()){
                    turret.shooterActivated = false;
                    follower.followPath(Path3);
                    setPathState(PathState.SHOOT2);

                }

                break;
            case SHOOT2:
                if(!follower.isBusy()){
                    intake.intakeOn = true;
                    intake.stopOn = false;
                    turret.shooterActivated = true;
                    if(pathTimer.seconds() > 2.5) {
                        intake.intakeOn = true;
                        intake.stopOn = true;
                        follower.followPath(Path4,1,false);
                        setPathState(PathState.GRABCORNER1);
                    }
                }
                if(pathTimer.seconds()>.5 && pathTimer.seconds() < 1){
                    turret.shooterActivated = true;
                    intake.intakeOn = false;

                }
                break;
            case GRABCORNER1:
                if(!follower.isBusy()){
                    turret.shooterActivated = false;
                    intake.stopOn = true;
                    follower.followPath(Path5,true);
                    if(pathTimer.seconds()>3.5 && pathTimer.seconds() < 3.55) {
                        intake.intakeOn = false;
                        intake.Outtake = true;
                    }
                    if(pathTimer.seconds() >= 3.55){
                        intake.Outtake = false;
                        setPathState(PathState.SHOOT3);
                    }
                }
                break;
            case SHOOT3:


                if(!follower.isBusy()){
                    follower.followPath(Path6);
                    setPathState(PathState.GRABCORNER2);
                    if(pathTimer.seconds()>.5){
                        turret.shooterActivated = true;
                    }
                }
                break;
            case GRABCORNER2:
                if(!follower.isBusy()){
                    if(pathTimer.seconds()> .5 && pathTimer.seconds()<2.5) {
                        intake.intakeOn = true;
                        intake.stopOn = false;
                        turret.shooterActivated = true;
                    }
                    if(pathTimer.seconds() >= 2.6) {
                        intake.intakeOn = true;
                        intake.stopOn = true;
                        follower.followPath(Path4,1,false);
                        setPathState(PathState.SHOOT4);
                    }

                }

                if(pathTimer.seconds()>.5 && pathTimer.seconds() < 1.5){
                    turret.shooterActivated = true;
                    intake.intakeOn = false;

                }
                break;
            case SHOOT4:
                if(!follower.isBusy()){
                    turret.shooterActivated = false;
                    intake.stopOn = true;
                    follower.followPath(Path5,true);
                    if(pathTimer.seconds()>3.5 && pathTimer.seconds() < 3.55) {
                        intake.intakeOn = false;
                        intake.Outtake = true;
                    }
                    if(pathTimer.seconds() >= 3.55){
                        intake.Outtake = false;
                        setPathState(PathState.GRABCORNER3);
                    }
                }
                break;
            case GRABCORNER3:
                if(!follower.isBusy()){
                    intake.stopOn = true;
                    follower.followPath(Path6);
                    setPathState(PathState.SHOOT5);
                    if(pathTimer.seconds()>.5){
                        turret.shooterActivated = true;
                    }
                }
                break;
            case SHOOT5:
                if(!follower.isBusy()){
                    if(pathTimer.seconds()> .5 && pathTimer.seconds()<2.5) {
                        turret.shooterActivated = true;
                        intake.intakeOn = true;
                        intake.stopOn = false;
                    }
                    if(pathTimer.seconds() > 2.6) {
                        intake.stopOn = true;
                        intake.intakeOn = true;
                        follower.followPath(Path7);
                        setPathState(PathState.LEAVE);
                    }
                }
                if(pathTimer.seconds()>1 && pathTimer.seconds() < 1.5){
                    turret.shooterActivated = true;
                    intake.intakeOn = false;

                }
                break;
            case LEAVE:
                if(!follower.isBusy()){
                    turret.shooterActivated = false;
                    intake.stopOn = true;
                    follower.followPath(Path8);
                    setPathState(PathState.MORE);
                    if(pathTimer.seconds()>1){
                        turret.shooterActivated = true;
                    }
                }
                break;
            case MORE:
                if(!follower.isBusy()){
                    turret.shooterActivated = true;
                    intake.intakeOn = true;
                    intake.stopOn = false;
                    if(pathTimer.seconds() > 2.6) {
                        intake.stopOn = true;
                        intake.intakeOn = true;
                        follower.followPath(Path12);
                        setPathState(PathState.LESS);
                    }
                }
                if(pathTimer.seconds()>1 && pathTimer.seconds() < 1.5){
                    turret.shooterActivated = true;
                    intake.intakeOn = false;

                }
                break;
            case LESS:
                if(!follower.isBusy()){
                    intake.stopOn = true;
                    follower.followPath(Path13);
                    setPathState(PathState.HOME);
                    if(pathTimer.seconds()>1){
                        turret.shooterActivated = true;
                    }
                }
                break;
            case HOME:
                if(!follower.isBusy()){
                    turret.shooterActivated = true;
                    intake.intakeOn = true;
                    intake.stopOn = false;
                    if(pathTimer.seconds() > 2.5) {
                        intake.intakeOn = true;
                        intake.stopOn = false;
//                        follower.followPath(Path14);
                        setPathState(PathState.ENDING);
                    }
                }
                if(pathTimer.seconds()>1 && pathTimer.seconds() < 1.5){
                    turret.shooterActivated = true;
                    intake.intakeOn = false;

                }
                break;
            case ENDING:
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
        turret.update(turret.turretpositionX(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),turret.turretpositionY(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),Math.toDegrees(follower.getHeading()),turret.redGoalX,turret.redGoalY,follower.getVelocity().getXComponent(),follower.getVelocity().getYComponent(),true,false);
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
        setPathState(PathState.SHOOT);
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
        drive.imu.recalibrateIMU();


        //state variables
        intake.intakeOn = false;
        intake.Outtake = false;
        intake.stopOn = false;


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
        setPathState(PathState.SHOOT);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        fileManager.FileWrite(follower.getPose().getX(),follower.getPose().getY(),follower.getHeading());
    }



}