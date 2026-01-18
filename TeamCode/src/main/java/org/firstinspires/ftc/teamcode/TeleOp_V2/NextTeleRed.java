package org.firstinspires.ftc.teamcode.TeleOp_V2;


import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SubIntake;
import org.firstinspires.ftc.teamcode.subsystems.SubShoot;
import org.firstinspires.ftc.teamcode.subsystems.SubTurret;


import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.ServoEx;


@TeleOp(name = "NextFTC TeleOp Program Red")
public class NextTeleRed extends NextFTCOpMode {
    public NextTeleRed() {
        addComponents(
                new SubsystemComponent(SubShoot.INSTANCE, SubIntake.INSTANCE, SubTurret.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    // change the names and directions to suit your robot



    //Turret Aiming variables:
    private static final double TURRET_POWER_MAX = 0.8;
    private static final double TURRET_POWER_MIN = 0.2;
    private static final double Kp = 0.040; // increase for raster response, decrease if it oscillates
    private static final double Ki = 0.0001; // increase if it stops early, decrease if it overshoots
    private static final double Kd = 0.001; // increase to reduce overshoot and oscillation, decrease if its too sluggish in moving
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();
    private static final double TARGET_TOLERANCE = 2.0;
    private ElapsedTime searchElapsedTimer = new ElapsedTime();
    private boolean autoTrackingEnabled = false;
    private LimelightSubsystem limelight;

    public static Pose startingPose = new Pose(8, 8, Math.toRadians(0));
    public static Pose BLUEGOAL = new Pose(156, 139, Math.toRadians(0));
    public double DISTANCETOBLUEGOAL;
    public double turnage;
    double botposeY;
    double botposeX;
    double botposeHeading;
    double shootertune;
    double HoodTune;
    double limepower;
    private ServoEx RGBLight = new ServoEx("RGB");
    private ServoEx RGBLight2 = new ServoEx("RGB2");

    // red - 0.28
    // green - 0.500
    // azure - 0.55
    //Indigo - 0.666
    // Violet - 0.722




    // 1150 rpm motor has encoder resolution of 145.1
    private double calculatePID(double error) { //PID calculations for setting turret motor aiming power
        // Proportional term
        double P = Kp * error;

        // Integral term
        integralSum += error * pidTimer.seconds();
        double I = Ki * integralSum;

        // Derivative term
        double D = Kd * (error - lastError) / pidTimer.seconds();

        pidTimer.reset();
        lastError = error;

        // Calculate total power
        double power = P + I + D;

        // limit power to max values
        if (Math.abs(power) > TURRET_POWER_MAX) {
            power = Math.signum(power) * TURRET_POWER_MAX;
        } else if (Math.abs(error) > TARGET_TOLERANCE && Math.abs(power) < TURRET_POWER_MIN) {
            power = Math.signum(power) * TURRET_POWER_MIN;
        } else if (Math.abs(error) <= TARGET_TOLERANCE) {
            power = 0; // Stop when on target
        }

        return power;
    }





    @Override
    public void onInit() {
        telemetry.setMsTransmissionInterval(11);
        RGBLight.setPosition(0.722);
        RGBLight2.setPosition(0.722);
        limelight = new LimelightSubsystem(hardwareMap);
        pidTimer.reset();
        PedroComponent.follower().setStartingPose(startingPose);
        telemetry.addData("pos", SubTurret.INSTANCE.getPosition());
        telemetry.update();






    }
    @Override
    public void onWaitForStart(){
        RGBLight.setPosition(0.722);
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
//        Command driverControlled = new MecanumDriverControlled(
//                frontLeftMotor.brakeMode(),
//                frontRightMotor.brakeMode(),
//                backLeftMotor.brakeMode(),
//                backRightMotor.brakeMode(),
//                Gamepads.gamepad1().leftStickY().negate(),
//                Gamepads.gamepad1().leftStickX(),
//                Gamepads.gamepad1().rightStickX()
//        );
//        driverControlled.schedule();




//        Gamepads.gamepad2().x()
//                .whenBecomesTrue(SubShoot.INSTANCE.PIDshot);
//                //.whenBecomesFalse(SubShoot.INSTANCE.PIDstop);
//        //.whenBecomesFalse(SubShoot.INSTANCE.CloseLaunch);


//        Gamepads.gamepad2().a()
//                .whenBecomesTrue(() -> SubTurret.INSTANCE.AIMER())  // Call the method
//                .whenBecomesFalse(SubTurret.INSTANCE.TestRun);



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


//        Gamepads.gamepad2().a()
//                .whenBecomesTrue(SubTurret.INSTANCE.AIMER())
//                .whenBecomesFalse(SubTurret.INSTANCE.TestRun);


//        telemetry.addData("Target Visible", targetVisible ? "YES" : "NO");
//        telemetry.addData("Auto tracking", autoTrackingEnabled ? "Enabled" : "Disabled");
//        if (!autoTrackingEnabled || !limelight.isTargetFound()) {
//            double manualPower = gamepad2.right_stick_x * TURRET_POWER_MAX;
//            TurretMotor.setPower(manualPower);
//            integralSum = 0;
//            lastError = 0;
//            pidTimer.reset();
//            telemetry.addData("Search mode", "MANUAL");
//        } else if (autoTrackingEnabled && limelight.isTargetFound()) {
//            double yawError = limelight.getYawAngle();
//            double turretPower = calculatePID(yawError);
//            TurretMotor.setPower(turretPower);
//            telemetry.addData("Search mode", "AUTO-TRACKING");
//            if (Math.abs(yawError) < TARGET_TOLERANCE) {
//                telemetry.addData("Status", "✓ ON TARGET!");
//                TurretMotor.setPower(0);
//            }
//        }
//        if (gamepad2.dpad_down){
//            double currentMotorVelocity = ShooterMotor.getVelocity(); // Get velocity in ticks/second
//            double motorPower = motorPIDVelocity.calculate(600000, currentMotorVelocity);
//            ShooterMotor.setPower(motorPower);
//        }
//        if (gamepad2.a) {
//            autoTrackingEnabled = true;
//        }
//        if (gamepad2.b) {
//            autoTrackingEnabled = false;
//        }
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
        turnage = (CorrectTurning/360) * 145.1 * 3.43;

        // turnticks =

        SubTurret.INSTANCE.setTarget(turnage);
        DISTANCETOBLUEGOAL = PedroComponent.follower().getPose().distanceFrom(BLUEGOAL);
//        telemetry.addData("turret turnage", turnage);
        //telemetry.addData("RobotPose", PedroComponent.follower().getPose());
        telemetry.addData("TurretPos", SubTurret.INSTANCE.getPosition());
        telemetry.addData("flywheelvel", SubShoot.INSTANCE.getvel());
        //telemetry.addData("Distance to blue goal", DISTANCETOBLUEGOAL);
        //telemetry.addData("Hood Pos", HoodTune);
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


        if (gamepad2.a ) {
            // Button just pressed - schedule the command with the current target
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