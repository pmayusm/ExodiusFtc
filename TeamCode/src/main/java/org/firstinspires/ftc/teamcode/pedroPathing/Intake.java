package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.hardware.impl.ServoEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private final DcMotorEx intakeMotor;
    private Servo Blocker;
    private final Command myLambdaCommand;
    private final Command BlockerCommand;

    // Constructor: pass in hardwareMap
    public Intake(HardwareMap hardwareMap) {
        // Initialize motor first
        intakeMotor = hardwareMap.get(DcMotorEx.class, "I");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Blocker = hardwareMap.get(Servo.class, "Blocker");



        BlockerCommand = new LambdaCommand()
                .setStart(() -> Blocker.setPosition(0.15)) // full power
                .setUpdate(() -> Blocker.setPosition(0.15)) // keep full power
                .setStop(interrupted -> Blocker.setPosition(1)) // stop motor
                .setIsDone(() -> false) // never finishes on its own
                .setInterruptible(true)
                .named("Blocker Command");

        // LambdaCommand that runs until manually stopped
        myLambdaCommand = new LambdaCommand()
                .setStart(() -> intakeMotor.setPower(1)) // full power
                .setUpdate(() -> intakeMotor.setPower(1)) // keep full power
                .setStop(interrupted -> intakeMotor.setPower(0)) // stop motor
                .setIsDone(() -> false) // never finishes on its own
                .setInterruptible(true)
                .named("Intake Command");




    }

    // Schedule the command
    public void runIntake() {
        myLambdaCommand.start();
    }

    // Stop the shooter manually if needed
    public void stopIntake() {
        myLambdaCommand.cancel();// stops the motor via setStop()
        intakeMotor.setPower(0);
    }
    public void BLOCK(){
        Blocker.setPosition(0.1);
    }
    public void UNBLOCK(){
        Blocker.setPosition(1);
    }

    public Command getCommand() {
        return myLambdaCommand;
    }
}
