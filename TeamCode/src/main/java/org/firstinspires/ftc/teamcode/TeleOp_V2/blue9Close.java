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
import org.firstinspires.ftc.teamcode.subsystems.SubTurret;
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

@Autonomous(name = "blue9Close")
public class blue9Close extends NextFTCOpMode {
    private LimelightSubsystem limelight;
    public blue9Close() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(SubShoot.INSTANCE, SubIntake.INSTANCE, SubTurret.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    //private Follower follower;
    public static Pose BLUEGOAL = new Pose(4, 144, Math.toRadians(0));

    double turnage;
    double shootertune;
    double DISTANCETOBLUEGOAL;
    double HoodTune;
    private final Pose startPose = new Pose(21, 123.5, Math.toRadians(235)); //starting pose
    private final Pose ScorePoseBigTriangle = new Pose(55, 90, Math.toRadians(180)); //first scoring spot at the big triangle
    private final Pose FirstIntake = new Pose(20, 82, Math.toRadians(180)); //Ending spot of first stack intake
    private final Pose SecondIntake = new Pose(10, 55, Math.toRadians(180)); //Ending spot of second stack intake
    private final Pose ParkPose = new Pose(45, 33, Math.toRadians(180)); //Parking spot at the end of Auto
    private PathChain scorePreload;
    private Path grabPickup1;
    private Path scorePickup1;
    private Path grabPickup2;
    private Path scorePickup2;
    private Path park;

    public void buildPaths() {

        scorePreload = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, ScorePoseBigTriangle))
                .setLinearHeadingInterpolation(startPose.getHeading(), ScorePoseBigTriangle.getHeading())
                .build();

        grabPickup1 = new Path(new BezierCurve(ScorePoseBigTriangle, new Pose(55, 70), FirstIntake));
        grabPickup1.setLinearHeadingInterpolation(ScorePoseBigTriangle.getHeading(), FirstIntake.getHeading());

        scorePickup1 = new Path(new BezierLine(FirstIntake, ScorePoseBigTriangle));
        scorePickup1.setLinearHeadingInterpolation(FirstIntake.getHeading(), ScorePoseBigTriangle.getHeading());

        grabPickup2 = new Path(new BezierCurve(ScorePoseBigTriangle,new Pose(60.000, 60.000), new Pose(47.000, 56.000),SecondIntake));
        grabPickup2.setConstantHeadingInterpolation(Math.toRadians(180));

        scorePickup2 = new Path(new BezierLine(SecondIntake, ScorePoseBigTriangle));
        scorePickup2.setConstantHeadingInterpolation(Math.toRadians(180));

        park = new Path(new BezierLine(ScorePoseBigTriangle, ParkPose));
        park.setConstantHeadingInterpolation(Math.toRadians(180));
    }


    private Command autonomousRoutine() {
        return new SequentialGroup(
                SubIntake.INSTANCE.HoldIntake,
                new FollowPath(scorePreload, false).asDeadline(
                        SubTurret.INSTANCE.AutonAim
                ),
                SubIntake.INSTANCE.StopIntake.and(SubIntake.INSTANCE.KickDown),
                SubIntake.INSTANCE.HoldIntake,
                new Delay(2),
                SubIntake.INSTANCE.KickUp,
                new Delay(0.2),
                new FollowPath(grabPickup1),
                new Delay(1),
                new FollowPath(scorePickup1),
                SubIntake.INSTANCE.StopIntake.and(SubIntake.INSTANCE.KickDown),
                SubIntake.INSTANCE.HoldIntake,
                new Delay(2),
                SubIntake.INSTANCE.KickUp,
                new Delay(0.2),
                new FollowPath(grabPickup2),
                new Delay(0.5),
                new FollowPath(scorePickup2),
                SubIntake.INSTANCE.StopIntake.and(SubIntake.INSTANCE.KickDown),
                SubIntake.INSTANCE.HoldIntake,
                new Delay(1.5),
                SubIntake.INSTANCE.KickUp

        );
    }
    private Command Initialize(){
        return new SequentialGroup(
                SubIntake.INSTANCE.KickUp,
                SubIntake.INSTANCE.HoldIntake,
                SubIntake.INSTANCE.StopIntake,
                SubShoot.INSTANCE.StopShoot

        );
    }

    @Override
    public void onInit(){
        limelight = new LimelightSubsystem(hardwareMap, 20);
        PedroComponent.follower().setStartingPose(startPose);

        // Set limelight reference in turret subsystem

        Initialize().schedule();
    }

    @Override
    public void onStartButtonPressed() {

        buildPaths();
        follower().update();
        autonomousRoutine().schedule();

    }
    @Override
    public void onUpdate(){
        SubShoot.INSTANCE.setPIDTRUE(true);
        SubShoot.INSTANCE.PIDshot.schedule();

        telemetry.addData("Hood Pos", SubShoot.INSTANCE.getHoodtune());
        telemetry.addData("Flywheel vel", SubShoot.INSTANCE.getvel());
        telemetry.addData("TargetFlywheel", SubShoot.INSTANCE.getTargetvelocity());
        telemetry.addData("turre pos", SubTurret.INSTANCE.getPosition());
        telemetry.addData("Robot Pos", PedroComponent.follower().getPose().toString());
        DISTANCETOBLUEGOAL = PedroComponent.follower().getPose().distanceFrom(BLUEGOAL);
        double dx = BLUEGOAL.getX() - PedroComponent.follower().getPose().getX();
        double dy = BLUEGOAL.getY() - PedroComponent.follower().getPose().getY();
        double fieldAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = Math.toDegrees(PedroComponent.follower().getHeading());
        double turretTargetAngle = fieldAngleToGoal - robotHeading;
        double CorrectTurning = normalizeAngle(turretTargetAngle);
        turnage = (CorrectTurning/360) * 145.1 * 3.1;
        SubTurret.INSTANCE.setTarget(turnage);
        shootertune = (5.25858 * DISTANCETOBLUEGOAL) + 788;
        SubShoot.INSTANCE.setTargetvelocity(shootertune);
        SubShoot.INSTANCE.sethoodtune(HoodTune);
        SubShoot.INSTANCE.HoodInterpolation().schedule();
        //SubShoot.INSTANCE.InterpolationTuning().schedule();
        //SubTurret.INSTANCE.AIMER().schedule();
        if (DISTANCETOBLUEGOAL >= 65.00){
            HoodTune = 0.35;
        }else if (DISTANCETOBLUEGOAL < 65.00){
            HoodTune = 0.8;
        }
        // turnticks =

        SubTurret.INSTANCE.setTarget(turnage);
        telemetry.update();
    }
    double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}