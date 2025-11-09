package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.ParallelGroup;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;


@Autonomous(name = "Example Auto", group = "Examples")
public class ExampleAuto extends OpMode {
    private double convertx = 0;

    private double converty = 0;
    private LimelightSubsystem limelight;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private Intake shooter;
    private Intake intake;
    private Command shooterIntakeParallel;




    private int pathState;

    private final Pose startPose = new Pose(21, 123, Math.toRadians(323)); // Start Pose of our robot.
    private final Pose scorePose1 = new Pose(59, 85, Math.toRadians(315)); // Scoring Pose at big triangle
    private final Pose pickup1Pose = new Pose(50, 84, Math.toRadians(180)); // preparing to intake first set of balls

    private final Pose picking = new Pose(21, 84, Math.toRadians(180)); // intaking first set of balls

    private final Pose scorePose2 = new Pose(61, 14, Math.toRadians(298));  // score pose from small triangle

    private final Pose humanintake = new Pose(125, 12, Math.toRadians(180)); // spot where robot takes balls from human player in observation zone

    private Path scorePreload;
    private PathChain prepare1, pickup1, score2, pickup2;

    private ElapsedTime TimePassed = new ElapsedTime();

    public void ConvertCoordinates(){
        if (limelight.isTargetFound()){
            converty = (39.3701 * limelight.getBotposeX()) +  72;
            convertx = (39.3701 * limelight.getBotposeZ()) + 72;
        }
    }

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose1));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading());
        scorePreload.setTimeoutConstraint(2);


    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        prepare1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup1Pose.getHeading())
                .build();

        pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, picking))
                .setConstantHeadingInterpolation(picking.getHeading())
                .setTimeoutConstraint(1)
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(picking, new Pose(63, 90), scorePose2))
                .setLinearHeadingInterpolation(picking.getHeading(), scorePose2.getHeading())
                .setTimeoutConstraint(1)
                .build();
        pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, humanintake))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), humanintake.getHeading())
                .setTimeoutConstraint(1)
                .build();


        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */


    }



    public void autonomousPathUpdate() {
        follower.update();
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, false);
                telemetry.addData("pos", follower.getPose());
                telemetry.update();
                break;
            case 1:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds()>2) {
                        shooterIntakeParallel.cancel();
                        telemetry.addData("path 1", true);
                        follower.followPath(prepare1, false);
                        setPathState(2);
                    }
                    break;
                }

            case 2:
                if (!follower.isBusy()){
                    follower.followPath(pickup1, false);
                    setPathState(3);
                    break;
                }
            case 3:
                if (!follower.isBusy()){
                    follower.followPath(score2, false);
                    setPathState(4);
                    break;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(pickup2, false);
                    break;
                }



//
//            /* You could check for
//            - Follower State: "if(!follower.isBusy()) {}"
//            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
//            - Robot Position: "if(follower.getPose().getX() > 36) {}"
//            */
//
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if (!follower.isBusy()) {
//                    /* Score Preload */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup1, true);
//                    setPathState(2);
//                }
//                break;
//            case 2:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
//                if (!follower.isBusy()) {
//                    /* Grab Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(grabPickup1, true);
//                    setPathState(3);
//                }
//                break;

        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {
        limelight = new LimelightSubsystem(hardwareMap, 20);

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

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}