package org.firstinspires.ftc.teamcode.TeleOp_V2;


import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SubIntake;
import org.firstinspires.ftc.teamcode.subsystems.SubShoot;
import org.firstinspires.ftc.teamcode.subsystems.SubTurret;


import java.io.File;
import java.util.ArrayList;
import java.util.List;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.ServoEx;


@TeleOp(name = "NextFTC TeleOp Program Java")
public class NextTele extends NextFTCOpMode {
    public NextTele() {
        addComponents(
                new SubsystemComponent(SubShoot.INSTANCE, SubIntake.INSTANCE, SubTurret.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    // change the names and directions to suit your robot





    private LimelightSubsystem limelight;
    private List<Integer> routine;

    public static Pose startingPose = new Pose(8, 8, Math.toRadians(0));
    public static Pose BLUEGOAL = new Pose(8, 139, Math.toRadians(0));
    public double DISTANCETOBLUEGOAL;
    public double turnage;
    double botposeY;
    double botposeX;
    double botposeHeading;
    double shootertune;
    double HoodTune;

    private ServoEx RGBLight = new ServoEx("RGB");
    private ServoEx RGBLight2 = new ServoEx("RGB2");

    // red - 0.28
    // green - 0.500
    // azure - 0.55
    //Indigo - 0.666
    // Violet - 0.722










    @Override
    public void onInit() {
        routine = new ArrayList<>();
        File closeAuto = AppUtil.getInstance().getSettingsFile("turretpos.txt");
        String[] types = ReadWriteFile.readFile(closeAuto).trim().split(", ");
        for (String type : types){
            if (!type.isEmpty()){
                routine.add(Integer.parseInt(type));
            }
        }
        telemetry.addData("routine", routine);
        telemetry.setMsTransmissionInterval(11);
        RGBLight.setPosition(0.722);
        RGBLight2.setPosition(0.722);
        limelight = new LimelightSubsystem(hardwareMap);
        PedroComponent.follower().setStartingPose(startingPose);
        telemetry.addData("pos", SubTurret.INSTANCE.getPosition());
        telemetry.update();
    }
    @Override
    public void onWaitForStart(){
        RGBLight.setPosition(0.722);
        if (gamepad2.leftBumperWasPressed()){
            SubTurret.INSTANCE.ResetTurret();
        }
        telemetry.addData("pos", SubTurret.INSTANCE.getPosition());
        telemetry.update();
    }
    @Override
    public void onStartButtonPressed() {
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate()
        );
        driverControlled.schedule();

        Gamepads.gamepad2().rightBumper()
                .whenBecomesTrue(SubIntake.INSTANCE.KickDown.and(SubIntake.INSTANCE.transferIntake))
                .whenBecomesFalse(SubIntake.INSTANCE.StopIntake.and(SubIntake.INSTANCE.KickUp));
        Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(SubIntake.INSTANCE.KickDown.and(SubIntake.INSTANCE.HoldIntake))
                .whenBecomesFalse(SubIntake.INSTANCE.KickUp.and(SubIntake.INSTANCE.StopIntake));
        Gamepads.gamepad1().rightTrigger().greaterThan(0.2)
                .whenBecomesTrue(SubIntake.INSTANCE.HoldIntake)
                .whenBecomesFalse(SubIntake.INSTANCE.StopIntake);

        Gamepads.gamepad1().leftTrigger().greaterThan(0.2)
                .whenBecomesTrue(SubIntake.INSTANCE.ReverseIntake)
                .whenBecomesFalse(SubIntake.INSTANCE.StopIntake);
        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(
                        SubIntake.INSTANCE.KickMiddle
                );
        Gamepads.gamepad1().x()
                .whenBecomesTrue(SubShoot.INSTANCE.hood1);
    }
    @Override
    public void onUpdate(){
        telemetry.addData("routine", routine);
        PedroComponent.follower().update();
        limelight.getLatestResult();
        limelight.update();
        boolean targetVisible = limelight.isTargetFound();
        if (targetVisible){
            telemetry.addData("tag visible", "true");
            telemetry.addData("LimeX",( limelight.getBotposeX() * 39.37) * -1 + 72);
            telemetry.addData("LimeZ", (limelight.getBotposeZ()* 39.37) * -1 + 72);
            telemetry.addData("LimeY", (limelight.getBotposeY()* 39.37) * -1 + 72);
            telemetry.addData("LimeYaw", limelight.getBotYaw());
            telemetry.addData("tx", limelight.getYawAngle());
        }
        else {
            telemetry.addData("tag visible", "false");
        }
        if (gamepad2.x){
            SubShoot.INSTANCE.setPIDTRUE(true);
        }
        if (!gamepad2.x){
            SubShoot.INSTANCE.setPIDTRUE(false);
        }

        double dx = BLUEGOAL.getX() - PedroComponent.follower().getPose().getX();
        double dy = BLUEGOAL.getY() - PedroComponent.follower().getPose().getY();
        double fieldAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = Math.toDegrees(PedroComponent.follower().getHeading());
        double turretTargetAngle = fieldAngleToGoal - robotHeading;
        double CorrectTurning = normalizeAngle(turretTargetAngle);
        turnage = (CorrectTurning/360) * 145.1 * 3.1;



        SubTurret.INSTANCE.setTarget(turnage);
        DISTANCETOBLUEGOAL = PedroComponent.follower().getPose().distanceFrom(BLUEGOAL);
        telemetry.addData("TurretPos", SubTurret.INSTANCE.getPosition());
        telemetry.addData("flywheelvel", SubShoot.INSTANCE.getvel());
        telemetry.addData("Robot Pose", PedroComponent.follower().getPose());
        telemetry.addData("CorrectTurningAngle", CorrectTurning);
        telemetry.addData("targ vel", SubShoot.INSTANCE.getTargetvelocity());

        // shooter vel :y = 0.000140673x^3 - 0.0615182x^2 + 12.28422x + 469.8692
        // hood pos: y = -0.00000594867x^3 + 0.00178147x^2 - 0.172839x + 5.77029
        shootertune = 0.000140673 * Math.pow(DISTANCETOBLUEGOAL, 3) - 0.0615182 * Math.pow(DISTANCETOBLUEGOAL, 2) + 12.28422 * DISTANCETOBLUEGOAL + 429.8692;
        HoodTune = -0.00000594867 * Math.pow(DISTANCETOBLUEGOAL, 3) + 0.00178147 * Math.pow(DISTANCETOBLUEGOAL, 2) - 0.172839 * DISTANCETOBLUEGOAL+ 5.77029;
        SubShoot.INSTANCE.setTargetvelocity(shootertune);
        SubShoot.INSTANCE.sethoodtune(HoodTune);
        SubShoot.INSTANCE.HoodInterpolation().schedule();
        telemetry.update();

        if (targetVisible){
            botposeX = (-39.37 * limelight.getBotposeX()) + 72 ;
            botposeY = (-39.37 * limelight.getBotposeZ()) + 72 ;
            botposeHeading = limelight.getBotYaw() - 90;
        }


        if (gamepad2.a) {
            SubTurret.INSTANCE.AIMER().schedule();
        }

        if (gamepad2.x){
            SubShoot.INSTANCE.setPIDTRUE(true);
            SubShoot.INSTANCE.InterpolationTuning().schedule();
        } else if (!gamepad2.x){
            SubShoot.INSTANCE.setPIDTRUE(false);
        }
        if ((Math.abs(SubShoot.INSTANCE.getvel() - shootertune))<= 50 ){
            RGBLight.setPosition(0.722);
            RGBLight2.setPosition(0.722);
            telemetry.addData("Velocity reached", true);
        } else {
            RGBLight.setPosition(0.28);
            RGBLight2.setPosition(0.28);
            telemetry.addData("velocity not reached", false);
        }
        if (gamepad1.dpadDownWasPressed()){
            PedroComponent.follower().setPose(startingPose);
        }

        if (targetVisible && gamepad1.leftBumperWasPressed()){
            PedroComponent.follower().setPose(getRobotPoseFromCamera());
        }

    }
    private Pose getRobotPoseFromCamera(){
        return new Pose(botposeX, botposeY, Math.toRadians(botposeHeading));
    }


    double normalizeAngle(double angle) {
        angle = -1 * (180 - angle);
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

}