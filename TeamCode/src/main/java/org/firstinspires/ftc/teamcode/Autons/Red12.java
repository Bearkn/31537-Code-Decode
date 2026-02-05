package org.firstinspires.ftc.teamcode.Autons;


import com.pedropathing.follower.Follower;
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
public class Red12 extends OpMode{
    private Follower follower;

    Shooter shooter = new Shooter();

    Intake intake = new Intake();

    Turret turret = new Turret();

    FileManager fileManager = new FileManager();

    private ElapsedTime pathTimer = new ElapsedTime();


    //state variables


    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(20, 10, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.


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
                .addPath(new BezierLine(startPose.getPose(),scorePose.getPose()))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose.getPose(),new Pose(40, -10)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), 0)
                .build();
//
//        path3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Pose(54,12),scorePose.getPose()))
//                .setLinearHeadingInterpolation(Math.toRadians(0), scorePose.getHeading())
//                .build();
//
//        path4 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose.getPose(),new Pose(20, -12)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(0))
//                .build();
//
//        path5 = follower.pathBuilder()
//                .addPath(new BezierLine(new Pose(20,-12),new Pose(72, -12)))
//                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(0))
//                .build();
//
//
//        path6 = follower.pathBuilder()
//                .addPath(new BezierLine(new Pose(54,-12),scorePose.getPose()))
//                .setLinearHeadingInterpolation(Math.toRadians(0), scorePose.getHeading())
//                .build();
//
//        path7 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose.getPose(),new Pose(20, -36)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(0))
//                .build();
//
//        path8 = follower.pathBuilder()
//                .addPath(new BezierLine(new Pose(20,-36),new Pose(70, -36)))
//                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(0))
//                .build();
//
//        path9 = follower.pathBuilder()
//                .addPath(new BezierLine(new Pose(56,-36),new Pose(36, 0)))
//                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(0))
//                .build();



    }

    public void autonomousPathUpdate() {
        switch (Pathstate) {
            case FIRST:

                if (!follower.isBusy()) {
                    follower.followPath(path1);
                    setPathState(PathState.SETUPGATE);
                }

                break;
            case SHOOT1:
                if (!follower.isBusy()) {
                    follower.followPath(path2);
                    setPathState(PathState.SETUPGATE);
                }
                break;
            case SETUPGATE:
                if(follower.isBusy()){
                    fileManager.FileWrite(follower.getPose().getX(),follower.getPose().getY(),follower.getHeading());
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
            if(shooter.currentFlywheelSpeed >= Math.abs(shooter.targetFlywheelSpeed-20) && Math.abs(turret.analogangle - (turret.turretAngle*360) ) < 5) {
                intake.stopState = Intake.StopState.SHOOT;
            }
        } else {
            intake.stopState = Intake.StopState.HOLD;
        }

        follower.update();
        autonomousPathUpdate();
        turret.update(turret.turretpositionX(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),turret.turretpositionY(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),Math.toDegrees(follower.getHeading()),turret.redGoalX,turret.redGoalY,follower.getVelocity().getXComponent(),follower.getVelocity().getYComponent(),true);
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
        setPathState(PathState.FIRST);
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
        setPathState(PathState.FIRST);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}


}