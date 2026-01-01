package org.firstinspires.ftc.teamcode.TeleOp_V2;


import com.arcrobotics.ftclib.command.Command;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MotorPIDVelocity;
import org.firstinspires.ftc.teamcode.subsystems.SubIntake;
import org.firstinspires.ftc.teamcode.subsystems.SubShoot;
import org.firstinspires.ftc.teamcode.subsystems.SubTurret;


import java.time.Duration;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;


@TeleOp(name = "interpolation testing")
public class interpolationtest extends NextFTCOpMode {
    public interpolationtest() {
        addComponents(
                new SubsystemComponent(SubShoot.INSTANCE, SubIntake.INSTANCE, SubTurret.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    // change the names and directions to suit your robot
    private final MotorEx frontLeftMotor = new MotorEx("FL").reversed().brakeMode();
    private final MotorEx frontRightMotor = new MotorEx("FR").brakeMode();
    private final MotorEx backLeftMotor = new MotorEx("BL").reversed().brakeMode();
    private final MotorEx backRightMotor = new MotorEx("BR").brakeMode();

    private final MotorEx TurretMotor = new MotorEx("TE").brakeMode().reversed();
    private final MotorEx ShooterMotor = new MotorEx("SH").zeroed();


    //Turret Aiming variables:
    private static final double TURRET_POWER_MAX = 1;
    private static final double TURRET_POWER_MIN = 0.3;
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
    public static Pose BLUEGOAL = new Pose(10, 138, Math.toRadians(0));
    public double DISTANCETOBLUEGOAL;
    public double turnage;
    public double shootertune;


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
        limelight = new LimelightSubsystem(hardwareMap, 20);
        SubShoot.INSTANCE.initialize();
        SubIntake.INSTANCE.initialize();
        SubTurret.INSTANCE.initialize();
        pidTimer.reset();
        PedroComponent.follower().setStartingPose(startingPose);



    }
    @Override
    public void onWaitForStart(){

    }
    @Override
    public void onStartButtonPressed() {
        TurretMotor.zeroed();
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
        limelight.update();
        limelight.getLatestResult();





//        Gamepads.gamepad2().a()
//                .whenBecomesTrue(() -> SubTurret.INSTANCE.AIMER())  // Call the method
//                .whenBecomesFalse(SubTurret.INSTANCE.TestRun);



        Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(SubIntake.INSTANCE.KickDown.and(SubIntake.INSTANCE.HoldIntake))
                .whenBecomesFalse(SubIntake.INSTANCE.KickUp.and(SubIntake.INSTANCE.StopIntake));
        Gamepads.gamepad2().rightBumper()
                .whenBecomesTrue(SubIntake.INSTANCE.KickDown.and(SubIntake.INSTANCE.transferIntake))
                .whenBecomesFalse(SubIntake.INSTANCE.StopIntake.and(SubIntake.INSTANCE.KickUp));
        Gamepads.gamepad1().rightTrigger().greaterThan(0.2)
                .whenBecomesTrue(SubIntake.INSTANCE.HoldIntake)
                .whenBecomesFalse(SubIntake.INSTANCE.StopIntake);

        Gamepads.gamepad1().leftTrigger().greaterThan(0.2)
                .whenBecomesTrue(SubIntake.INSTANCE.ReverseIntake)
                .whenBecomesFalse(SubIntake.INSTANCE.StopIntake);



    }
    @Override
    public void onUpdate(){
        limelight.getLatestResult();
        limelight.update();
        boolean targetVisible = limelight.isTargetFound();

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

        if (gamepad2.a) {
            // Button just pressed - schedule the command with the current target
            SubTurret.INSTANCE.AIMER().schedule();
        } else if (!gamepad2.a) {
            // Button released - run TestRun
            SubTurret.INSTANCE.TestRun.schedule();
        }



        double dx = BLUEGOAL.getX() - PedroComponent.follower().getPose().getX();
        double dy = BLUEGOAL.getY() - PedroComponent.follower().getPose().getY();
        double fieldAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = Math.toDegrees(PedroComponent.follower().getHeading());
        double turretTargetAngle = fieldAngleToGoal - robotHeading;
        double CorrectTurning = normalizeAngle(turretTargetAngle);
        turnage = (CorrectTurning/360) * 145.1 * 3.1;
        SubTurret.INSTANCE.setTarget(turnage * -1);

        DISTANCETOBLUEGOAL = PedroComponent.follower().getPose().distanceFrom(BLUEGOAL);


        telemetry.addData("flywheelvel", SubShoot.INSTANCE.getvel());
        telemetry.addData("Distance to blue goal", DISTANCETOBLUEGOAL);
        telemetry.addData("target velocity", SubShoot.INSTANCE.getTargetvelocity());
        telemetry.update();
        //shootertune = 0.0945044 * (Math.pow(DISTANCETOBLUEGOAL, 2)) - 3.75527 * DISTANCETOBLUEGOAL + 1884.5904;
        shootertune = 0.00206249 * (Math.pow(DISTANCETOBLUEGOAL, 3)) - 0.642804 * (Math.pow(DISTANCETOBLUEGOAL, 2 )) + 68.4677 * DISTANCETOBLUEGOAL - 1083.84987;
        SubShoot.INSTANCE.setTargetvelocity(shootertune);

//        if (gamepad2.aWasPressed()){
//            shootertune += 50;
//        }
//        if (gamepad2.bWasPressed()){
//            shootertune -= 50;
//        }
        if (gamepad2.x){
            SubShoot.INSTANCE.setPIDTRUE(true);
            SubShoot.INSTANCE.InterpolationTuning().schedule();
        } else if (!gamepad2.x){
            SubShoot.INSTANCE.setPIDTRUE(false);
        }
        //0.00206249x^3 - 0.642804x^2 + 68.4677x - 1083.84987

    }
    double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}