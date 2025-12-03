package org.firstinspires.ftc.teamcode.TeleOp_V2;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.util.Timer;




import org.firstinspires.ftc.teamcode.pedroPathing.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.ParallelGroup;


@TeleOp(name = "TeleOp V2")
public class TeleOpV2  extends LinearOpMode {
    double denom;
    private LimelightSubsystem limelight;



    // Declare the Limelight subsystem



    static MecanumDrive drive;
    static GamepadEx gamepad1Ex;
    static GamepadEx gamepad2Ex;
    static RobotV2 robot;

    private static final double TURRET_POWER_MAX = 1;
    private static final double TURRET_POWER_MIN = 0.3;
    private static final double SEARCH_POWER = 0.4;

    // PID Constants for turret aiming
    private static final double Kp = 0.040; // increase for raster response, decrease if it oscillates
    private static final double Ki = 0.0001; // increase if it stops early, decrease if it overshoots
    private static final double Kd = 0.001; // increase to reduce overshoot and oscillation, decrease if its too sluggish in moving

    // PID tracking variables
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();

    // Target tolerance (degrees from center)
    private static final double TARGET_TOLERANCE = 5.0;

    // Search mode variables
    private static final double SEARCH_TIMEOUT = 3.0;
    private double searchTimer = 0;
    private int searchDirection = 1; // 1 for clockwise, -1 for counter-clockwise
    private ElapsedTime searchElapsedTimer = new ElapsedTime();


    // Control flags
    private boolean autoTrackingEnabled = false;
    private boolean lastAState = false;
    private boolean lastBState = false;



//    private ColorSensor colorSensor;;
//    private double redValue;
//    private double greenValue;
//    private double blueValue;
//    private double alphaValue; //Light Intensity
//    private double TargetValue = 1000;
//private ColorSensor colorSensor;;






    Intake intake;

    Shooter shooter;

//    public void initColorSensor(){
//        colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");
//    }
//    public void getColor(){
//        redValue = colorSensor.red();
//        greenValue = colorSensor.green();
//        blueValue = colorSensor.blue();
//        alphaValue = colorSensor.alpha();
//    }
//    public void colorTelemetry(){
//        telemetry.addData("redValue",redValue);
//        telemetry.addData("greenValue", greenValue);
//        telemetry.addData("blueValue", blueValue);
//        telemetry.addData("alphaValue", alphaValue);
//        telemetry.update();
//    }






    private void HardwareStart() {
        //follower = Constants.createFollower(hardwareMap);
        //follower.setStartingPose(startPose);
        robot = new RobotV2();
        robot.init(hardwareMap);
        drive = new MecanumDrive(robot.FrontLeft, robot.FrontRight, robot.BackLeft, robot.BackRight);
        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);
        limelight = new LimelightSubsystem(hardwareMap, 20);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        pidTimer.reset();
        searchElapsedTimer.reset();
        //robot.TurretEncoderReset();
        //robot.encoder();
        //initColorSensor();

        // Init Actions



    }
    //gamepad1, right bumber goes to close than open


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the Limelight subsystem to track AprilTag ID 20


        // Initialization telemetry
        telemetry.addData("Status", "Initialized");
        //telemetry.addData("Target April Tag", limelight.getTargetAprilTagId());
        telemetry.update();
        ElapsedTime TimePassed = new ElapsedTime();
        int v_state = 0;

        waitForStart();
        HardwareStart();
        waitForStart();
        //getColor();
        //colorTelemetry();

        while (opModeIsActive()) {
            CommandManager.INSTANCE.run();
            limelight.update();
            limelight.getLatestResult();
            robot.SH2.set(gamepad2Ex.getLeftY());
            robot.SH.set(gamepad2Ex.getLeftY());
            drive.driveRobotCentric(
                    gamepad1Ex.getLeftX(),
                    gamepad1Ex.getLeftY(),
                    gamepad1Ex.getRightX(),
                    true


            );

            // CRITICAL: Update the Limelight data every loop
            limelight.update();
            limelight.getLatestResult();

            // Get the pitch angle from the subsystem


            // Check if the target is found
            boolean targetVisible = limelight.isTargetFound();



            // Display the pitch angle in telemetry
            telemetry.addData("==== LIMELIGHT DATA ====", "");
            telemetry.addData("Target Visible", targetVisible ? "YES" : "NO");

            //getColor();
            //colorTelemetry();
//            switch (v_state)
//            {
//                case 0:
//                    robot.Coax.setPosition(1);
//                    TimePassed.reset();
//                    v_state++;
//                    break;
//                case 1:
//                    if (TimePassed.time() >= 2.0)
//                    {
//                        robot.IntakeClaw.setPosition(1);
//                        v_state = 0;
//                    }
//            }
//           STATE MACHINE FOR WAITS ABOVE
//           COLOR SENSOR BELOW
//            if(alphaValue > targetValue){

//                run action
//            } else {
//                run other action
//            }

            //start tele here

            if (!limelight.isTargetFound()){
                searchElapsedTimer.reset();
            }

            // Manual turret control with right stick (overrides auto-tracking)
            if (!autoTrackingEnabled) {
                double manualPower = gamepad2Ex.getRightX() * TURRET_POWER_MAX;
                robot.TurretEnc.setPower(manualPower);

                // Reset PID when manually controlling
                integralSum = 0;
                lastError = 0;

                telemetry.addData("Mode", "MANUAL OVERRIDE");
                telemetry.addData("Manual Power", "%.2f", manualPower);
                if (limelight.isTargetFound()){
                    telemetry.addData("Yaw error", limelight.getYawAngle());
                }

            } else if (autoTrackingEnabled) {
                // Auto-tracking mode
                if (limelight.isTargetFound()) {
                    // Target found - use PID to aim
                    double yawError = limelight.getYawAngle();
                    double turretPower = calculatePID(yawError);
                    robot.TurretEnc.setPower(turretPower);

                    // Reset search timer
                    searchTimer = 0;
                    searchElapsedTimer.reset();

                    telemetry.addData("Mode", "AUTO-TRACKING");
                    //telemetry.addData("Target ID", limelight.getTargetAprilTagId());
                    telemetry.addData("Yaw Error", "%.2f°", yawError);
                    telemetry.addData("Pitch Angle", "%.2f°", limelight.getPitchAngle());
                    telemetry.addData("Turret Power", "%.3f", turretPower);

                    if (Math.abs(yawError) < TARGET_TOLERANCE) {
                        telemetry.addData("Status", "✓ ON TARGET!");
                        robot.TurretEnc.setPower(0);
                    } else {
                        telemetry.addData("Status", "Aiming...");
                    }

                    // Display bot pose information
                    telemetry.addData("Bot X", "%.2f", limelight.getBotposeX());
                    telemetry.addData("Bot Z", "%.2f", limelight.getBotposeZ());
                    telemetry.addData("Bot Yaw", "%.2f°", limelight.getBotYaw());

                } else if (robot.TurretEnc.getCurrentPosition() > 600 || robot.TurretEnc.getCurrentPosition() < -600) {
                    autoTrackingEnabled = !autoTrackingEnabled;

                }
                else if (!limelight.isTargetFound()){
                    // No target - search mode

                    //searchTimer += searchElapsedTimer.seconds();
                    //searchElapsedTimer.reset();
//                    if (searchElapsedTimer.time() < 0.3){
////                        robot.TurretEnc.setPower(0);
////                    }

                    //robot.TurretEnc.setPower(SEARCH_POWER * searchDirection);
                    telemetry.addData("Mode", "SEARCHING");
                    //robot.TurretEnc.setPower(0);
                    // Reset PID values
                    integralSum = 0;
                    lastError = 0;
                    //telemetry.addData("Target ID", limelight.getTargetAprilTagId());
                    //telemetry.addData("Search Direction", searchDirection > 0 ? "Clockwise" : "Counter-Clockwise");
                    //telemetry.addData("Search Timer", "%.1f s", searchTimer);
                    telemetry.addData("Status", "Spinning to find target...");
                }
            }



            if (gamepad2Ex.getButton(GamepadKeys.Button.B)){
                intake.runIntake();
            } else {
                intake.stopIntake();
            }
//            if (gamepad2Ex.getButton(GamepadKeys.Button.A)){
//                intake.UNBLOCK();
//            }else {
//                intake.BLOCK();
//            }
            if (gamepad2Ex.getButton(GamepadKeys.Button.A)){
                autoTrackingEnabled = !autoTrackingEnabled;
                if (!autoTrackingEnabled) {
                    //robot.TurretSpin.set(0);
                    integralSum = 0;
                    lastError = 0;
                }
            }
//            if (gamepad1Ex.getButton(GamepadKeys.Button.X)){
//                intake.testCS();
//            }
//            if (gamepad2Ex.getButton(GamepadKeys.Button.X)){
//                shooter.runShooter();
//            }
//            if (gamepad2Ex.getButton(GamepadKeys.Button.Y)){
//                shooter.stopShooter();
//            }
//            if (gamepad2Ex.getButton(GamepadKeys.Button.LEFT_BUMPER)){
//                if (!robot.TurretEnc.isBusy()) {
//                    double yawerror = limelight.getLatestResult().getTx();
//                    if (yawerror > 5.0) {
//                        robot.turnToAprilTag(yawerror);
//                    }
//                }
//            }












            telemetry.addData("Turret mode:", autoTrackingEnabled ? "auto":"manual");
            telemetry.addData("Search timer", searchElapsedTimer.time());
            telemetry.update();


        }
    }
    private double calculatePID(double error) {
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

        // Clamp power to max values
        if (Math.abs(power) > TURRET_POWER_MAX) {
            power = Math.signum(power) * TURRET_POWER_MAX;
        } else if (Math.abs(error) > TARGET_TOLERANCE && Math.abs(power) < TURRET_POWER_MIN) {
            power = Math.signum(power) * TURRET_POWER_MIN;
        } else if (Math.abs(error) <= TARGET_TOLERANCE) {
            power = 0; // Stop when on target
        }

        return power;
    }


}



