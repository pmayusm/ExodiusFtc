package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.utility.LambdaCommand;

public class Shooter {

    private final DcMotorEx shooterMotor;
    private final Command myLambdaCommand;

    // Constructor: pass in hardwareMap
    public Shooter(HardwareMap hardwareMap) {
        // Initialize motor first
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // LambdaCommand that runs until manually stopped
        myLambdaCommand = new LambdaCommand()
                .setStart(() -> shooterMotor.setPower(1)) // full power
                .setUpdate(() -> shooterMotor.setPower(1)) // keep full power
                .setStop(interrupted -> shooterMotor.setPower(0)) // stop motor
                .setIsDone(() -> false) // never finishes on its own
                .setInterruptible(true)
                .named("Shooter Command");



    }

    // Schedule the command
    public void runShooter() {
        CommandManager.INSTANCE.scheduleCommand(myLambdaCommand);
    }

    // Stop the shooter manually if needed
    public void stopShooter() {
        myLambdaCommand.cancel(); // stops the motor via setStop()
    }

    public Command getCommand() {
        return myLambdaCommand;
    }
}
