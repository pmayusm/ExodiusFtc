package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.utility.LambdaCommand;

public class Intake {

    private final DcMotorEx intakeMotor;
    private final Command myLambdaCommand;

    // Constructor: pass in hardwareMap
    public Intake(HardwareMap hardwareMap) {
        // Initialize motor first
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // LambdaCommand that runs until manually stopped
        myLambdaCommand = new LambdaCommand()
                .setStart(() -> intakeMotor.setPower(1)) // full power
                .setUpdate(() -> intakeMotor.setPower(1)) // keep full power
                .setStop(interrupted -> intakeMotor.setPower(0)) // stop motor
                .setIsDone(() -> false) // never finishes on its own
                .setInterruptible(true)
                .named("Shooter Command");



    }

    // Schedule the command
    public void runIntake() {
        CommandManager.INSTANCE.scheduleCommand(myLambdaCommand);
    }

    // Stop the shooter manually if needed
    public void stopIntake() {
        myLambdaCommand.cancel(); // stops the motor via setStop()
    }

    public Command getCommand() {
        return myLambdaCommand;
    }
}
