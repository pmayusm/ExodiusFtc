package org.firstinspires.ftc.teamcode.TeleOp_V2;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SubIntake;
import org.firstinspires.ftc.teamcode.subsystems.SubShoot;
import org.firstinspires.ftc.teamcode.subsystems.SubTurret;
import com.qualcomm.robotcore.util.ReadWriteFile;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

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

import androidx.annotation.RequiresPermission;


@Autonomous(name = "NextFTC Autonomous Program Java")
public class AutonomousProgram extends NextFTCOpMode {
    private LimelightSubsystem limelight;
    public AutonomousProgram() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(SubShoot.INSTANCE, SubIntake.INSTANCE, SubTurret.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }


    private List<Integer> routine;
    public static Pose BLUEGOAL = new Pose(4, 144, Math.toRadians(0));
    public double turretPosition;
    public int roundedPos;

    double turnage;
    double shootertune;
    double DISTANCETOBLUEGOAL;
    double HoodTune;
    private final Pose startPose = new Pose(21, 123.5, Math.toRadians(235)); //starting pose
    private final Pose ScorePoseBigTriangle = new Pose(55, 90, Math.toRadians(197)); //first scoring spot at the big triangle
    private final Pose FirstIntake = new Pose(20, 82, Math.toRadians(180)); //Ending spot of first stack intake
    private final Pose Gate = new Pose(18, 75, Math.toRadians(180));  // Spot to open the gate
    private final Pose SecondIntake = new Pose(10, 59, Math.toRadians(180)); //Ending spot of second stack intake
    private final Pose ThirdIntake = new Pose(10, 35, Math.toRadians(180));
    private final Pose ParkPose = new Pose(27, 75, Math.toRadians(270)); //Parking spot at the end of Auto
    private PathChain scorePreload;
    private Path grabPickup1;
    private Path OpenGate;
    private Path scorePickup1;
    private Path grabPickup2;
    private Path scorePickup2;
    private Path grabPickup3;
    private Path scorePickup3;
    private Path park;

    public void buildPaths() {


        scorePreload = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, ScorePoseBigTriangle))
                .setLinearHeadingInterpolation(startPose.getHeading(), ScorePoseBigTriangle.getHeading())
                .setTimeoutConstraint(300)
                .build();

        grabPickup1 = new Path(new BezierCurve(ScorePoseBigTriangle, new Pose(55, 70), FirstIntake));
        grabPickup1.setLinearHeadingInterpolation(ScorePoseBigTriangle.getHeading(), FirstIntake.getHeading());

        OpenGate = new Path(new BezierCurve(FirstIntake, new Pose(31, 73), Gate));
        OpenGate.setLinearHeadingInterpolation(FirstIntake.getHeading(), Gate.getHeading());

        scorePickup1 = new Path(new BezierLine(Gate, ScorePoseBigTriangle));
        scorePickup1.setLinearHeadingInterpolation(Gate.getHeading(), ScorePoseBigTriangle.getHeading());

        grabPickup2 = new Path(new BezierCurve(ScorePoseBigTriangle,new Pose(70.000, 58.000), new Pose(47.000, 50.000),SecondIntake));
        grabPickup2.setLinearHeadingInterpolation(ScorePoseBigTriangle.getHeading(), SecondIntake.getHeading());

        scorePickup2 = new Path(new BezierCurve(SecondIntake, new Pose(30, 35), ScorePoseBigTriangle));
        scorePickup2.setLinearHeadingInterpolation(SecondIntake.getHeading(), ScorePoseBigTriangle.getHeading());

        grabPickup3 = new Path(new BezierCurve(ScorePoseBigTriangle, new Pose(75, 15), ThirdIntake));
        grabPickup3.setLinearHeadingInterpolation(ScorePoseBigTriangle.getHeading(), ThirdIntake.getHeading());

        scorePickup3 = new Path(new BezierLine(ThirdIntake, ScorePoseBigTriangle));
        scorePickup3.setLinearHeadingInterpolation(ThirdIntake.getHeading(), ScorePoseBigTriangle.getHeading());

        park = new Path(new BezierLine(ScorePoseBigTriangle, ParkPose));
        park.setLinearHeadingInterpolation(ScorePoseBigTriangle.getHeading(), ParkPose.getHeading());
    }


    private Command autonomousRoutine() {
        return new SequentialGroup(
                new FollowPath(scorePreload, false).and(SubTurret.INSTANCE.AutonAim),
                SubIntake.INSTANCE.KickDown.and(SubIntake.INSTANCE.HoldIntake),
                new Delay(1.8),
                SubIntake.INSTANCE.KickUp,
                new Delay(0.2),
                new FollowPath(grabPickup1),
                new Delay(0.3),
                new FollowPath(OpenGate),
                new Delay(0.2),
                new FollowPath(scorePickup1).and(SubIntake.INSTANCE.StopIntake),
                SubIntake.INSTANCE.KickDown.and(SubIntake.INSTANCE.HoldIntake),
                new Delay(1.8),
                SubIntake.INSTANCE.KickUp,
                new Delay(0.2),
                new FollowPath(grabPickup2),
                new Delay(0.3),
                new FollowPath(scorePickup2).and(SubIntake.INSTANCE.StopIntake),
                SubIntake.INSTANCE.HoldIntake.and(SubIntake.INSTANCE.KickDown),
                new Delay(1.8),
                SubIntake.INSTANCE.KickUp,
                new FollowPath(grabPickup3),
                new Delay(0.3),
                new FollowPath(scorePickup3).and(SubIntake.INSTANCE.StopIntake),
                SubIntake.INSTANCE.KickDown.and(SubIntake.INSTANCE.HoldIntake),
                new Delay(1.8),
                SubIntake.INSTANCE.KickUp,
                new Delay(0.2),
                new FollowPath(park).and(SubTurret.INSTANCE.TestRun)


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
        routine = new ArrayList<>();
        SubTurret.INSTANCE.ResetTurret();
        limelight = new LimelightSubsystem(hardwareMap);
        PedroComponent.follower().setStartingPose(startPose);
        // Set limelight reference in turret subsystem
        Initialize().schedule();
    }

    @Override
    public void onStartButtonPressed() {
        routine.add(19);
        buildPaths();
        PedroComponent.follower().update();
        autonomousRoutine().schedule();
    }
    @Override
    public void onUpdate(){
        SubShoot.INSTANCE.setPIDTRUE(true);
        SubShoot.INSTANCE.PIDshot.schedule();
        telemetry.addData("routine", routine);
        telemetry.addData("Hood Pos", SubShoot.INSTANCE.getHoodtune());
        telemetry.addData("Flywheel vel", SubShoot.INSTANCE.getvel());
        telemetry.addData("turret pos", SubTurret.INSTANCE.getPosition());
        telemetry.addData("Robot Pos", PedroComponent.follower().getPose().toString());
        telemetry.addData("turret Position", SubTurret.INSTANCE.getPosition());
        DISTANCETOBLUEGOAL = PedroComponent.follower().getPose().distanceFrom(BLUEGOAL);
        double dx = BLUEGOAL.getX() - PedroComponent.follower().getPose().getX();
        double dy = BLUEGOAL.getY() - PedroComponent.follower().getPose().getY();
        double fieldAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = Math.toDegrees(PedroComponent.follower().getHeading());
        double turretTargetAngle = fieldAngleToGoal - robotHeading;
        double CorrectTurning = normalizeAngle(turretTargetAngle);
        turnage = (CorrectTurning/360) * 145.1 * 3.4;
        SubTurret.INSTANCE.setTarget(turnage);
        shootertune = (5.25858 * DISTANCETOBLUEGOAL) + 788;
        SubShoot.INSTANCE.setTargetvelocity(shootertune);
        SubShoot.INSTANCE.sethoodtune(HoodTune);
        HoodTune = -0.00000594867 * Math.pow(DISTANCETOBLUEGOAL, 3) + 0.00178147 * Math.pow(DISTANCETOBLUEGOAL, 2) - 0.172839 * DISTANCETOBLUEGOAL+ 5.77029;
        SubShoot.INSTANCE.HoodInterpolation().schedule();

        //SubShoot.INSTANCE.InterpolationTuning().schedule();
        //SubTurret.INSTANCE.AIMER().schedule();
        // turnticks =

        SubTurret.INSTANCE.setTarget(turnage);
        telemetry.update();

    }
    @Override public void onStop() {
        turretPosition = SubTurret.INSTANCE.getPosition();
        roundedPos = (int) turretPosition;
        routine.add(roundedPos);
        String routineString = routine.toString();
        routineString = routineString.substring(1, routineString.length() -1);
        File turretpos = AppUtil.getInstance().getSettingsFile("turretpos.txt");
        ReadWriteFile.writeFile(turretpos, routineString);

    }
    double normalizeAngle(double angle) {
        angle = -1 * (180 - angle);
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}