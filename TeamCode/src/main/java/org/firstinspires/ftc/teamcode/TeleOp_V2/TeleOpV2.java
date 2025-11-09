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

    private LimelightSubsystem limelight;

    private double convertx = 0;

    private double converty = 0;
    private final Pose scorePose1 = new Pose(59, 85, Math.toRadians(315)); // Scoring Pose at big triangle
    private Pose startPose = new Pose(21, 123, Math.toRadians(323)); // Start Pose of our robot.
    private Follower follower;
    private Path scoring;

    // Declare the Limelight subsystem



    static MecanumDrive drive;
    static GamepadEx gamepad1Ex;
    static GamepadEx gamepad2Ex;
    static RobotV2 robot;



//    private ColorSensor colorSensor;;
//    private double redValue;
//    private double greenValue;
//    private double blueValue;
//    private double alphaValue; //Light Intensity
//    private double TargetValue = 1000;
//private ColorSensor colorSensor;;


    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;


    private Path testpath;
    private double redValue;
    private double greenValue;
    private double blueValue;
    private double alphaValue; //Light Intensity
    private double targetValue = 1000;




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

    public void buildPaths(){
        testpath = new Path(new BezierLine(startPose, scorePose1));
        testpath.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading());
        testpath.setTimeoutConstraint(2);
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }




    private void HardwareStart() {
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        robot = new RobotV2();
        robot.init(hardwareMap);
        drive = new MecanumDrive(robot.FrontLeft, robot.FrontRight, robot.BackLeft, robot.BackRight);
        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        //initColorSensor();

        // Init Actions



    }
    //gamepad1, right bumber goes to close than open


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the Limelight subsystem to track AprilTag ID 20
        limelight = new LimelightSubsystem(hardwareMap, 20);

        // Initialization telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Target April Tag", limelight.getTargetAprilTagId());
        telemetry.update();
        ElapsedTime TimePassed = new ElapsedTime();
        int v_state = 0;

        waitForStart();
        HardwareStart();
        waitForStart();
        //getColor();
        //colorTelemetry();

        while (opModeIsActive()) {

            drive.driveRobotCentric(
                    gamepad1Ex.getLeftX(),
                    gamepad1Ex.getLeftY(),
                    gamepad1Ex.getRightX(),
                    true


            );

            // CRITICAL: Update the Limelight data every loop
            limelight.update();

            // Get the pitch angle from the subsystem
            double pitchAngle = limelight.getPitchAngle();

            // Check if the target is found
            boolean targetVisible = limelight.isTargetFound();

            if (limelight.isTargetFound()){
                converty = (39.3701 * limelight.getBotposeX()) +  72;
                convertx = (39.3701 * limelight.getBotposeZ()) + 72;
                Pose RobotPose = new Pose(convertx, converty, Math.toRadians(limelight.getBotYaw()));
                scoring = new Path(new BezierLine(startPose, scorePose1));
                scoring.setLinearHeadingInterpolation(RobotPose.getHeading(), scorePose1.getHeading());
                scoring.setTimeoutConstraint(2);
            }

            // Display the pitch angle in telemetry
            telemetry.addData("==== LIMELIGHT DATA ====", "");
            telemetry.addData("Target Visible", targetVisible ? "YES" : "NO");
            telemetry.addData("Pitch Angle", "%.2f degrees", pitchAngle);
            telemetry.addData("Yaw Angle", "%.2f degrees", limelight.getYawAngle());
            telemetry.addData("limelight botpose X", limelight.getBotposeX());
            telemetry.addData("limelight botpose Z", limelight.getBotposeZ());
            telemetry.addData("pedro pose X", convertx);
            telemetry.addData("pedro pose Y", converty);

            // Add helpful message if target not found
            if (!targetVisible) {
                telemetry.addData("Status", "Looking for target (April Tag)");
            } else {
                telemetry.addData("Status", "Found target(April tag)");
            }
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




            if (gamepad2Ex.getButton(GamepadKeys.Button.B)){
                intake.runIntake();
            } else {
                intake.stopIntake();
            }
            if (gamepad2Ex.getButton(GamepadKeys.Button.A)){
                intake.UNBLOCK();
            }else {
                intake.BLOCK();
            }
            if (gamepad2Ex.getButton(GamepadKeys.Button.X)){
                shooter.runShooter();
            }
            if (gamepad2Ex.getButton(GamepadKeys.Button.Y)){
                shooter.stopShooter();
            }
//            if (gamepad1Ex.getButton(GamepadKeys.Button.X) && limelight.isTargetFound()){
//
//                follower.followPath(scoring, false);
//                follower.update();
//            }
//            if (gamepad2Ex.getButton(GamepadKeys.Button.RIGHT_BUMPER) && gamepad2Ex.getButton(GamepadKeys.Button.DPAD_RIGHT)){
//                limelight.setTargetAprilTagId(24);
//            }
//            if (gamepad2Ex.getButton(GamepadKeys.Button.LEFT_BUMPER) && gamepad2Ex.getButton(GamepadKeys.Button.DPAD_LEFT)){
//                limelight.setTargetAprilTagId(20);
//            }

            telemetry.update();






           // telemetry.addData("Extension Position Variable", extpos);
            telemetry.addData("TimePassed", TimePassed.time());
            telemetry.update();


        }
    }
}



