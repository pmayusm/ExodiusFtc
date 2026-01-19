package org.firstinspires.ftc.teamcode.TeleOp_V2;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SubIntake;
import org.firstinspires.ftc.teamcode.subsystems.SubShoot;
import org.firstinspires.ftc.teamcode.subsystems.SubTurret;

import java.util.ArrayList;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "FarAutoBlue")
public class FarAutoBlue extends NextFTCOpMode {
    public FarAutoBlue(){
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(SubShoot.INSTANCE, SubIntake.INSTANCE, SubTurret.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }
    double DISTANCETOBLUEGOAL;
    double shootertune;
    double HoodTune;
    public final Pose startPose = new Pose(56, 8, Math.toRadians(180));
    public static Pose BLUEGOAL = new Pose(4, 144, Math.toRadians(0));

    public final Pose FirstIntake = new Pose(10, 9, Math.toRadians(180));
    public final Pose ScorePoseFar = new Pose(50, 12, Math.toRadians(180));
    private Path grabPickup1;
    private Path hesiPickup;
    private Path scorePickup1;

    public void buildPaths(){

        grabPickup1 = new Path(new BezierLine(startPose, FirstIntake));
        grabPickup1.setLinearHeadingInterpolation(startPose.getHeading(), FirstIntake.getHeading());

        hesiPickup = new Path(new BezierCurve(FirstIntake, new Pose(45, 8.5), FirstIntake));
        hesiPickup.setLinearHeadingInterpolation(FirstIntake.getHeading(), FirstIntake.getHeading());

        scorePickup1 = new Path(new BezierLine(FirstIntake, ScorePoseFar));
        scorePickup1.setLinearHeadingInterpolation(FirstIntake.getHeading(), ScorePoseFar.getHeading());
    }
    private Command autonomousRoutine(){
        return new SequentialGroup(
            SubTurret.INSTANCE.AutonFarAim1,
            new Delay(2.5),
            SubIntake.INSTANCE.HoldIntake.and(SubIntake.INSTANCE.KickDown),
            new Delay(1.5),
            SubIntake.INSTANCE.KickUp,
            new Delay(0.3),
            new FollowPath(grabPickup1).and(SubTurret.INSTANCE.AutonAimFar),
            new Delay(0.5),
            new FollowPath(scorePickup1),
            new Delay(0.5),
            SubIntake.INSTANCE.KickDown,
            new Delay(1.5),
            SubIntake.INSTANCE.KickUp,
            //go back again
            new FollowPath(grabPickup1),
            new Delay(0.5),
            new FollowPath(scorePickup1),
            new Delay(0.5),
            SubIntake.INSTANCE.KickDown,
            new Delay(1.5),
            SubIntake.INSTANCE.KickUp,
            // go back again
            new FollowPath(grabPickup1),
            new Delay(0.5),
            new FollowPath(scorePickup1),
            new Delay(0.5),
            SubIntake.INSTANCE.KickDown,
            new Delay(1.5),
            SubIntake.INSTANCE.KickUp,
            // go back again
            new FollowPath(grabPickup1),
            new Delay(0.5),
            new FollowPath(scorePickup1),
            new Delay(0.5),
            SubIntake.INSTANCE.KickDown,
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
        SubTurret.INSTANCE.ResetTurret();
        PedroComponent.follower().setPose(startPose);
        Initialize().schedule();
    }

    @Override
    public void onStartButtonPressed() {
        buildPaths();
        PedroComponent.follower().update();
        autonomousRoutine().schedule();
    }
    @Override
    public void onUpdate(){
        SubShoot.INSTANCE.setPIDTRUE(true);
        SubShoot.INSTANCE.PIDfarShot.schedule();
        telemetry.addData("Hood Pos", SubShoot.INSTANCE.getHoodtune());
        telemetry.addData("Flywheel vel", SubShoot.INSTANCE.getvel());
        telemetry.addData("turret pos", SubTurret.INSTANCE.getPosition());
        telemetry.addData("Robot Pos", PedroComponent.follower().getPose().toString());
        telemetry.addData("turret Position", SubTurret.INSTANCE.getPosition());
        DISTANCETOBLUEGOAL = PedroComponent.follower().getPose().distanceFrom(BLUEGOAL);
        HoodTune = -0.00000594867 * Math.pow(DISTANCETOBLUEGOAL, 3) + 0.00178147 * Math.pow(DISTANCETOBLUEGOAL, 2) - 0.172839 * DISTANCETOBLUEGOAL+ 5.77029;
        SubShoot.INSTANCE.sethoodtune(HoodTune);
        SubShoot.INSTANCE.HoodInterpolation().schedule();
        telemetry.update();

    }
    double normalizeAngle(double angle) {
        angle = -1 * (180 - angle);
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }


}
