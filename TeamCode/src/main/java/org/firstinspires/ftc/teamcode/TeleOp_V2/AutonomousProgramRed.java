package org.firstinspires.ftc.teamcode.TeleOp_V2;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SubIntake;
import org.firstinspires.ftc.teamcode.subsystems.SubShoot;
import org.firstinspires.ftc.teamcode.subsystems.TurretPIDSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

@Autonomous(name = "NextFTC Autonomous Program RED")
public class AutonomousProgramRed extends NextFTCOpMode {
    private LimelightSubsystem limelight;
    public AutonomousProgramRed() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(SubShoot.INSTANCE, SubIntake.INSTANCE, TurretPIDSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    //private Follower follower;

    private final Pose startPose = new Pose(123, 123.5, Math.toRadians(305)); //starting pose
    private final Pose ScorePoseBigTriangle = new Pose(81, 76, Math.toRadians(20)); //first scoring spot at the big triangle
    private final Pose FirstIntake = new Pose(116, 70, Math.toRadians(0)); //Ending spot of first stack intake
    private final Pose SecondIntake = new Pose(111, 50, Math.toRadians(0)); //Ending spot of second stack intake
    private final Pose ParkPose = new Pose(99, 33, Math.toRadians(0)); //Parking spot at the end of Auto
    private Path scorePreload;
    private Path grabPickup1;
    private Path scorePickup1;
    private Path grabPickup2;
    private Path scorePickup2;
    private Path park;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, ScorePoseBigTriangle));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), ScorePoseBigTriangle.getHeading());

        grabPickup1 = new Path(new BezierCurve(ScorePoseBigTriangle, new Pose(89, 70), FirstIntake));
        grabPickup1.setLinearHeadingInterpolation(ScorePoseBigTriangle.getHeading(), FirstIntake.getHeading());

        scorePickup1 = new Path(new BezierLine(FirstIntake, ScorePoseBigTriangle));
        scorePickup1.setLinearHeadingInterpolation(FirstIntake.getHeading(), ScorePoseBigTriangle.getHeading());

        grabPickup2 = new Path(new BezierCurve(ScorePoseBigTriangle,new Pose(84, 60.000), new Pose(97, 56.000),SecondIntake));
        grabPickup2.setConstantHeadingInterpolation(Math.toRadians(180));

        scorePickup2 = new Path(new BezierLine(SecondIntake, ScorePoseBigTriangle));
        scorePickup2.setConstantHeadingInterpolation(Math.toRadians(180));

        park = new Path(new BezierLine(ScorePoseBigTriangle, ParkPose));
        park.setConstantHeadingInterpolation(Math.toRadians(180));
    }


    private Command autonomousRoutine() {
        follower().setStartingPose(startPose);
        return new SequentialGroup(
                SubIntake.INSTANCE.KickMiddle,
                SubShoot.INSTANCE.AutoCloseShoot,
                SubShoot.INSTANCE.StopShoot,
                SubIntake.INSTANCE.HoldIntake,
                SubIntake.INSTANCE.StopIntake,
                new FollowPath(scorePreload, false),
                SubShoot.INSTANCE.AutoCloseShoot,
                TurretPIDSubsystem.INSTANCE.aimAtTarget(),
                new Delay(0.8),
                SubIntake.INSTANCE.KickUp,
                // first ball
                new Delay(0.4),
                SubIntake.INSTANCE.KickDown.and(SubIntake.INSTANCE.HoldIntake),
                new Delay(1.5),
                SubIntake.INSTANCE.StopIntake.and(SubIntake.INSTANCE.KickUp),
                // second ball
                new Delay(0.4),
                SubIntake.INSTANCE.KickDown.and(SubIntake.INSTANCE.HoldIntake),
                new Delay(1.5),
                SubIntake.INSTANCE.StopIntake.and(SubIntake.INSTANCE.KickUp),
                // third ball
                new Delay(0.4),
                SubIntake.INSTANCE.KickDown,
                SubShoot.INSTANCE.StopShoot,
                SubIntake.INSTANCE.HoldIntake,
                new FollowPath(grabPickup1, true, 0.5),
                new Delay(0.5),
                SubIntake.INSTANCE.StopIntake,
                SubShoot.INSTANCE.AutoCloseShoot,
                new FollowPath(scorePickup1, true),
                TurretPIDSubsystem.INSTANCE.aimAtTarget(),
                new Delay(1),
                SubIntake.INSTANCE.KickUp,
                // fourth ball
                new Delay(0.4),
                SubIntake.INSTANCE.KickDown,
                new Delay(0.1),
                SubIntake.INSTANCE.HoldIntake,
                new Delay(1.2),
                SubIntake.INSTANCE.StopIntake,
                SubIntake.INSTANCE.KickUp,
                // fifth ball
                SubIntake.INSTANCE.KickDown,
                SubIntake.INSTANCE.HoldIntake,
                new Delay(1.2),
                SubIntake.INSTANCE.KickUp
                // sixth ball


//
        );
    }
    private Command Initialize(){
        return new SequentialGroup(
                SubIntake.INSTANCE.KickUp,
                SubIntake.INSTANCE.KickDown
        );
    }

    @Override
    public void onInit(){
        limelight = new LimelightSubsystem(hardwareMap, 24);

        // Set limelight reference in turret subsystem
        TurretPIDSubsystem.INSTANCE.setLimelight(limelight);
        SubShoot.INSTANCE.initialize();
        SubIntake.INSTANCE.initialize();
        TurretPIDSubsystem.INSTANCE.initialize();
        Initialize().schedule();
    }

    @Override
    public void onStartButtonPressed() {
        telemetry.addData("Flywheel vel", SubShoot.INSTANCE.getvel());
        buildPaths();
        follower().update();
        autonomousRoutine().schedule();

    }
}