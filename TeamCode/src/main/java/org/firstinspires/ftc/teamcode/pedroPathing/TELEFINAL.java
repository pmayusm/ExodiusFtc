package org.firstinspires.ftc.teamcode.pedroPathing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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

import dev.nextftc.bindings.BindingManager;


import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

import java.util.function.Supplier;

@Configurable
@TeleOp

public class TELEFINAL extends OpMode {

    static GamepadEx gamepad1Ex;
    static GamepadEx gamepad2Ex;
    public double convertx;
    public double converty;
    int blockstate = 0;
    private LimelightSubsystem limelight;
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this

    private final Pose scorePose1 = new Pose(59, 85, Math.toRadians(315)); // Scoring Pose at big triangle


    private boolean automatedDrive;
    double ANGLEPOSE;
    private Supplier<PathChain> pedroteletest;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private final Pose BLUEGOAL = new Pose(13, 136, Math.toRadians(180)); // position of the blue goal
    private Intake intake;
    private Shooter shooter;

    @Override
    public void init() {
        limelight = new LimelightSubsystem(hardwareMap, 20);
        limelight.switchPipeline(9);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);


        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(59, 95))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 1))
                .build();

    }

    @Override
    public void start() {
        limelight = new LimelightSubsystem(hardwareMap, 20);
        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();

    }
    private Pose getFromLime(){
        return new Pose(convertx, converty, limelight.getBotYaw());

    }



    @Override
    public void loop() {
        limelight.update();
        follower.update();
        telemetryM.update();
        boolean targetVisible = limelight.isTargetFound();
        if (targetVisible) {
            ANGLEPOSE = 90 - limelight.getPitchAngle();
            convertx = 39.3701 * limelight.getBotposeZ() + 72;
            converty = 39.3701 * limelight.getBotposeX() + 72;
            if (gamepad2Ex.getButton(GamepadKeys.Button.Y)){
                shooter.setpos(0.58);
            }
            follower.setPose(getFromLime());
            telemetry.addData("Limelight Position:", follower.getPose());
            telemetry.addData("targpitch", ANGLEPOSE);
            telemetry.update();
        }
        BindingManager.update();
//        if (gamepad1Ex.getButton(GamepadKeys.Button.Y)){
//            follower.setPose(getFromLime());
//        }



        //Call this once per loop


        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }

        //Automated PathFollowing
//        if (gamepad1.aWasPressed()) {
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }

        if (gamepad1Ex.getButton(GamepadKeys.Button.X)){
            pedroteletest = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower.getPose(),scorePose1 )))
                    .setHeadingInterpolation(HeadingInterpolator.facingPoint(BLUEGOAL))
                    .setTimeoutConstraint(3)
                    .build();
            if (!follower.isBusy()) {
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), scorePose1))
                                .setHeadingInterpolation(HeadingInterpolator.facingPoint(BLUEGOAL))
                                .build()
                );
            }
            automatedDrive = true;
        }
//        if (gamepad1Ex.getButton(GamepadKeys.Button.LEFT_BUMPER)){
//            follower.followPath(pedroteletest.get());
//        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //blocks artifacts from going to shooter
        if (gamepad2Ex.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1){
            intake.runIntake();
        } else {
            intake.stopIntake();
        }

        if (gamepad2Ex.getButton(GamepadKeys.Button.A)){
            intake.BLOCK();
        }
        if (gamepad2Ex.getButton(GamepadKeys.Button.B)){
            intake.UNBLOCK();
        }
        if (gamepad2Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1){
            shooter.runShooter();
        } else {
            shooter.stopShooter();
        }
        if (gamepad2Ex.getButton(GamepadKeys.Button.X)){
            shooter.setpos(0.52);
        }

        if (gamepad2Ex.getButton(GamepadKeys.Button.DPAD_RIGHT)){
            //shooter.rot1();
            shooter.setpos(0.55);
        }




//        if (gamepad1.xWasPressed()) {
//            slowModeMultiplier += 0.25;
//        }
//
//
//        if (gamepad1.yWasPressed()) {
//            slowModeMultiplier -= 0.25;
//        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetry.addData("Target Visible", targetVisible ? "YES" : "NO");
        telemetryM.debug("lime X", convertx);
        telemetryM.debug("lime Y", converty);
        telemetryM.debug("targPitch",  90 -  limelight.getPitchAngle());
        telemetry.update();
        telemetryM.update(telemetry);
    }
}