package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.utility.LambdaCommand;

public class SubSystem {

    private final DcMotorEx intakeMotor;
    private final DcMotorEx shooterMotor;
    private final Command IntakeCommand;
    private final Command ShooterCommand;

    // Constructor: pass in hardwareMap
    public SubSystem(HardwareMap hardwareMap) {
        // Initialize motor first
        intakeMotor = hardwareMap.get(DcMotorEx.class, "I");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor = hardwareMap.get(DcMotorEx.class, "SH");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // LambdaCommand that runs until manually stopped
        IntakeCommand = new LambdaCommand()
                .setStart(() -> intakeMotor.setPower(1)) // full power
                .setUpdate(() -> intakeMotor.setPower(1)) // keep full power
                .setStop(interrupted -> intakeMotor.setPower(0)) // stop motor
                .setIsDone(() -> false) // never finishes on its own
                .setInterruptible(true)
                .named("Intake Command");

        ShooterCommand = new LambdaCommand()
                .setStart(() -> shooterMotor.setPower(1)) // full power
                .setUpdate(() -> shooterMotor.setPower(1)) // keep full power
                .setStop(interrupted -> shooterMotor.setPower(0)) // stop motor
                .setIsDone(() -> false) // never finishes on its own
                .setInterruptible(true)
                .named("Intake Command");



    }

    // Schedule the command
    public void runIntake() {
        CommandManager.INSTANCE.scheduleCommand(IntakeCommand);
    }

    // Stop the shooter manually if needed
    public void stopIntake() {
        IntakeCommand.cancel(); // stops the motor via setStop()
    }

    public Command getIntakeCommand() {
        return IntakeCommand;
    }
    public Command getShooterCommand(){
        return ShooterCommand;
    }
}