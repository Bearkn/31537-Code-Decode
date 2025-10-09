package org.firstinspires.ftc.teamcode.Autons;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Example Auto", group = "Examples")

public class BasicMoveAuton extends OpMode{
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState = 0;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(40, 0, Math.toRadians(90)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose nextPose = new Pose(40, 40, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose finalPose = new Pose(0, 40, Math.toRadians(270)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private Path move;
    private PathChain move2,move3,move4;


    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//        move = new Path(new BezierLine(startPose, scorePose));
//        move.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        move2 = follower.pathBuilder()
                .addPath(new BezierLine(startPose,scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .addPath(new BezierLine(scorePose, nextPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), nextPose.getHeading())
                .addPath(new BezierLine(nextPose, finalPose))
                .setLinearHeadingInterpolation(nextPose.getHeading(), finalPose.getHeading())
                .addPath(new BezierLine(finalPose, startPose))
                .setLinearHeadingInterpolation(finalPose.getHeading(), startPose.getHeading())
                .build();

//        move3 = follower.pathBuilder()
//                .addPath(new BezierLine(nextPose, finalPose))
//                .setLinearHeadingInterpolation(nextPose.getHeading(), finalPose.getHeading())
//                .addPath(new BezierLine(finalPose, startPose))
//                .setLinearHeadingInterpolation(finalPose.getHeading(), startPose.getHeading())
//                        .build();
//        move2 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, nextPose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), nextPose.getHeading())
//                .build();
//        move3 = follower.pathBuilder()
//                .addPath(new BezierLine(nextPose, finalPose))
//                .setLinearHeadingInterpolation(nextPose.getHeading(), finalPose.getHeading())
//                .build();
//        move4 = follower.pathBuilder()
//                .addPath(new BezierLine(finalPose, startPose))
//                .setLinearHeadingInterpolation(finalPose.getHeading(), startPose.getHeading())
//                .build();
//        follower.followPath(move3);
//        follower.followPath(move2);


    }

    public void autonomousPathUpdate() {
            switch (pathState) {
                case 0:
                    if(!follower.isBusy()) {
                        follower.followPath(move2,true);
//                        setPathState(1);
                        break;
                    }
                case 1:
                    if(!follower.isBusy()) {
                        follower.followPath(move3);
                        setPathState(0);
                        break;
                    }
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
            }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }
    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}
    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}
