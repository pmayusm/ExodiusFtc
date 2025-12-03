package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.SubIntake;
import org.firstinspires.ftc.teamcode.subsystems.SubShoot;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.ftc.components.BulkReadComponent;


@Autonomous(name = "BlueFarComp1Auto", group = "Examples")
public class TurretAutoDec6 extends OpMode {



    private SubShoot Shooter;
    private SubIntake Intake;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private final Pose startPose = new Pose(21, 123.5, Math.toRadians(235)); //starting pose
    private final Pose ScorePoseBigTriangle = new Pose(56, 88, Math.toRadians(180)); //first scoring spot at the big triangle
    private final Pose FirstIntake = new Pose(33, 80, Math.toRadians(180)); //Ending spot of first stack intake
    private final Pose SecondIntake = new Pose(33, 50, Math.toRadians(180)); //Ending spot of second stack intake
    private final Pose ParkPose = new Pose(45, 33, Math.toRadians(180)); //Parking spot at the end of Auto
    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, park;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, ScorePoseBigTriangle));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), ScorePoseBigTriangle.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(ScorePoseBigTriangle, new Pose(47, 82), FirstIntake))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(FirstIntake, ScorePoseBigTriangle))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(ScorePoseBigTriangle,new Pose(60.000, 60.000), new Pose(47.000, 56.000),SecondIntake))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(SecondIntake, ScorePoseBigTriangle))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(ScorePoseBigTriangle, ParkPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
    public void autonomousPathUpdate(){
        switch(pathState){
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {

                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;
                 /* You could check for
                - Follower State: "if(!follower.isBusy()) {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                 */
            case 2:
                if (!follower.isBusy()){
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()){
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()){
                    follower.followPath(park, true);
                    setPathState(-1);
                }
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



