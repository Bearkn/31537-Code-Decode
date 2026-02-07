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
import org.firstinspires.ftc.teamcode.mechanism.Shooter;
import org.firstinspires.ftc.teamcode.mechanism.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class Red18FAR extends OpMode{
    private Follower follower;

    Shooter shooter = new Shooter();

    Intake intake = new Intake();

    Turret turret = new Turret();

    FileManager fileManager = new FileManager();

    private ElapsedTime pathTimer = new ElapsedTime();


    //state variables


    private final Pose startPose = new Pose(24.5, -63, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(20, 10, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.


    public enum PathState {
        SHOOT,
        GRABCORNER,
        SHOOT1,
        GRAB3,
        MOVETOSHOOT,
        SHOOT2,
        GRABCORNER1,
        MOVETOSHOOT1,
        SHOOT3,
        GRABCORNER2,
        MOVETOSHOOT2,
        SHOOT4,
        GRABCORNER3,
        MOVETOSHOOT3,
        SHOOT5,
        LEAVE,
        ENDING
    }
    PathState Pathstate;
    public PathChain Grabcorner,GOSHOOT1,grab3,GOSHOOT2,GRABCORNER1,GOSHOOT3,GRABCORNER2,GOSHOOT4,GRABCORNER3,GOSHOOT5,LEAVE;

    public void buildPaths() {
        Grabcorner = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.5, -63),

                                new Pose(60, -63)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                    .addTemporalCallback(1500, () -> {setPathState(PathState.SHOOT1);})
                .build();

        GOSHOOT1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60, -65),

                                new Pose(18, -63)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        grab3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(18, 9.000-72),
                                new Pose(22, 40.000-72),
                                new Pose(61, 36.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        GOSHOOT2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(61, 36.000-72),

                                new Pose(18, 9.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        GRABCORNER1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18, 9.000-72),

                                new Pose(60, 8.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        GOSHOOT3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60, 8.000-72),

                                new Pose(18, 9.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        GRABCORNER2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(18, 9.000-72),
                                new Pose(33, 20.000-72),
                                new Pose(60, 20.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        GOSHOOT4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60, 20.000-72),

                                new Pose(18, 9.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        GRABCORNER3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18, 9.000-72),

                                new Pose(60, 8.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        GOSHOOT5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60, 8.000-72),

                                new Pose(18, 9.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        LEAVE = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18, 9.000-72),

                                new Pose(36, 9.000-72)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

    }

    public void autonomousPathUpdate() {
        switch (Pathstate) {
            case SHOOT:
                shooter.shooterActivated = true;
                if(pathTimer.seconds()>2){
                    intake.stopOn = false;
                }
                if(pathTimer.seconds()>4) {
                    intake.stopOn = true;
                    intake.intakeOn = true;
                    follower.followPath(Grabcorner,true);
                    setPathState((PathState.GRABCORNER));
                }
                break;

            case GRABCORNER:
                if (!follower.isBusy()) {
                    follower.followPath(GOSHOOT1,.7,true);
                    setPathState(PathState.SHOOT1);
                }
                break;
            case SHOOT1:
//                if(pathTimer.seconds()<.7){
//                    intake.intakeOn=true;
//                }
//                if(pathTimer.seconds()>=.7 && pathTimer.seconds()<3.5){
//                    intake.intakeOn=false;
//                }
                if(!follower.isBusy()){
                    intake.stopOn = false;
                    intake.intakeOn = false;
                    if(pathTimer.seconds() >=3.5) {
                        intake.stopOn = true;
                        intake.intakeOn = true;
                        setPathState(PathState.GRAB3);
                    }
                }
                break;
            case GRAB3:
                if(!follower.isBusy()){
                    follower.followPath(grab3);
                    setPathState(PathState.MOVETOSHOOT);
                }

                break;
            case MOVETOSHOOT:
                if(!follower.isBusy()){
                    follower.followPath(GOSHOOT2,.7,true);
                    setPathState(PathState.SHOOT2);
                }
            case SHOOT2:
                if(!follower.isBusy()){
                    intake.stopOn = false;
                    intake.intakeOn = false;
                    if(pathTimer.seconds() >=3.5) {
                        intake.stopOn = true;
                        intake.intakeOn = true;
                        setPathState(PathState.GRABCORNER1);
                    }
                }
                break;
            case GRABCORNER1:
                if(!follower.isBusy()){
                    follower.followPath(GRABCORNER1,.7,false);
                    setPathState(PathState.MOVETOSHOOT1);
                }
                break;
            case MOVETOSHOOT1:
                if(!follower.isBusy()){
                    follower.followPath(GOSHOOT3,.7,true);
                    setPathState(PathState.SHOOT3);
                }
            case SHOOT3:
                if(!follower.isBusy()){
                    intake.stopOn = false;
                    intake.intakeOn = false;
                    if(pathTimer.seconds() >=3.5) {
                        intake.stopOn = true;
                        intake.intakeOn = true;
                        setPathState(PathState.GRABCORNER2);
                    }
                }
                break;
            case GRABCORNER2:
                if(!follower.isBusy()){
                    follower.followPath(GRABCORNER2);
                    setPathState(PathState.MOVETOSHOOT2);
                }
                break;
            case MOVETOSHOOT2:
                if(!follower.isBusy()){
                    follower.followPath(GOSHOOT4,.7,true);
                    setPathState(PathState.SHOOT4);
                }
            case SHOOT4:
                if(!follower.isBusy()){
                    intake.stopOn = false;
                    intake.intakeOn = false;
                    if(pathTimer.seconds() >=3.5) {
                        intake.stopOn = true;
                        intake.intakeOn = true;
                        setPathState(PathState.GRABCORNER3);
                    }
                }
                break;
            case GRABCORNER3:
                if(!follower.isBusy()){
                    follower.followPath(GRABCORNER3);
                    setPathState(PathState.MOVETOSHOOT3);
                }
                break;
            case MOVETOSHOOT3:
                if(!follower.isBusy()){
                    follower.followPath(GOSHOOT3,.7,true);
                    setPathState(PathState.SHOOT5);
                }
            case SHOOT5:
                if(!follower.isBusy()){
                    intake.stopOn = false;
                    intake.intakeOn = false;
                    if(pathTimer.seconds() >=3.5) {
                        intake.stopOn = true;
                        intake.intakeOn = true;
                        setPathState(PathState.LEAVE);
                    }
                }
                break;
            case LEAVE:
                if(!follower.isBusy()){
                    follower.followPath(LEAVE,.6,true);
                    setPathState(PathState.ENDING);
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
                intake.intakeState = Intake.IntakeState.SHOOTFAR;
            }
            if(shooter.currentFlywheelSpeed >= Math.abs(shooter.targetFlywheelSpeed-100)) {
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
        telemetry.update();



    }
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        setPathState(PathState.SHOOT);
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        turret.init(hardwareMap);
        fileManager.init();
        pathTimer = new ElapsedTime();
        pathTimer.reset();
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
        setPathState(PathState.SHOOT);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        fileManager.FileWrite(follower.getPose().getX(),follower.getPose().getY(),follower.getHeading());
    }



}