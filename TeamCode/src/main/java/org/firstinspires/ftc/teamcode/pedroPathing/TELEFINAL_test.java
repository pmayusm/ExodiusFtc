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
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;


import org.firstinspires.ftc.teamcode.TeleOp_V2.RobotV2;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

import java.time.Duration;
import java.util.function.Supplier;


@TeleOp(name = "TELEFINAL_Test")
public class TELEFINAL_test extends OpMode {

    static GamepadEx gamepad1Ex;
    static GamepadEx gamepad2Ex;


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
            limelight.switchPipeline(9);
            follower = Constants.createFollower(hardwareMap);
            follower.update();
            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
            //new LimelightSubsystem(hardwareMap, 20);
            gamepad1Ex = new GamepadEx(gamepad1);
            gamepad2Ex = new GamepadEx(gamepad2);
            //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
            //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
            //If you don't pass anything in, it uses the default (false)
            follower.startTeleopDrive();
            intake = new Intake(hardwareMap);
            shooter = new Shooter(hardwareMap);

            robot = new RobotV2();
            robot.init(hardwareMap);


            telemetry.addData("Status", "Initialized Successfully");
            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("Init Error", e.getMessage());
            telemetry.update();
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        CommandManager.INSTANCE.run();
        limelight.getLatestResult();
        limelight.update();
        telemetry.addData("Status", "Running");
        telemetry.update();
        ElapsedTime TimePassed = new ElapsedTime();
        follower.update();
        telemetryM.update();
        boolean targetVisible = limelight.isTargetFound();
//        if (gamepad2Ex.getButton(GamepadKeys.Button.X)){
//            shooter.setpos(0.15);
//        }
//
//        if (gamepad2Ex.getButton(GamepadKeys.Button.DPAD_RIGHT)){
//            //shooter.rot1();
//            shooter.setpos(0.62);
//        }
        if (gamepad2Ex.getButton(GamepadKeys.Button.DPAD_RIGHT)){
            shooter.runShooter();
        }else {
            shooter.stopShooter();
        }
//        if (gamepad2Ex.getButton(GamepadKeys.Button.DPAD_DOWN)){
//            CommandManager.INSTANCE.scheduleCommand(
//                    new SequentialGroup(
//                            new InstantCommand(shooter.getCommand()),
//                            new Delay(1),
//                            new InstantCommand(intake.Feed())
////                            new ParallelGroup(
////                                    shooter.getCommand(),
////                                    new Delay(1)
////                            ),
////                            new ParallelGroup(
////                                    intake.Feed()
////                            )
//
//                    )
//            );
//        }
        if (gamepad2Ex.getButton(GamepadKeys.Button.B)){
            CommandManager.INSTANCE.scheduleCommand(intake.getCommand());
        }



    }
}