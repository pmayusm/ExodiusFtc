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
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;


import org.firstinspires.ftc.teamcode.TeleOp_V2.RobotV2;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

import java.time.Duration;
import java.util.function.Supplier;


@TeleOp
@Configurable

public class TELEFINAL extends OpMode {

    static GamepadEx gamepad1Ex;
    static GamepadEx gamepad2Ex;

    int blockstate = 0;
    private LimelightSubsystem limelight;
    private Follower follower;



    private boolean automatedDrive;
    double ANGLEPOSE;

    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private Intake intake;
    private Shooter shooter;
    static RobotV2 robot;




    @Override
    public void init() {
        try {
            limelight = new LimelightSubsystem(hardwareMap);
            follower = Constants.createFollower(hardwareMap);
            follower.update();
            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


            telemetry.addData("Status", "Initialized Successfully");
            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("Init Error", e.getMessage());
            telemetry.update();
        }



    }

    @Override
    public void start() {

        //new LimelightSubsystem(hardwareMap, 20);
        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();

    }




    @Override
    public void loop() {
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        robot = new RobotV2();
        robot.init(hardwareMap);

        limelight.switchPipeline(9);
        ElapsedTime TimePassed = new ElapsedTime();
        limelight.update();
        follower.update();
        telemetryM.update();
        boolean targetVisible = limelight.isTargetFound();

        //BindingManager.update();
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
        if (gamepad1Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1){
            intake.runIntake();
        } else {
            intake.stopIntake();
        }

//        if (gamepad2Ex.getButton(GamepadKeys.Button.A)){
//            intake.BLOCK();
//        }
//        if (gamepad2Ex.getButton(GamepadKeys.Button.B)){
//            intake.UNBLOCK();
//        }else {
//            intake.BLOCK();
//        }
        if (gamepad2Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1){
            shooter.runShooter();
        } else {
            shooter.stopShooter();
        }
        if (gamepad2Ex.getButton(GamepadKeys.Button.X)){
            shooter.setpos(0.42);
        }

        if (gamepad2Ex.getButton(GamepadKeys.Button.DPAD_RIGHT)){
            //shooter.rot1();
            shooter.setpos(0.62);
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
        telemetryM.debug("targPitch",  90 -  limelight.getPitchAngle());
        telemetry.update();
        telemetryM.update(telemetry);
    }
}