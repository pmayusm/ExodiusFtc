package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.hardware.impl.ServoEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.time.Duration;

public class Intake {

    private final DcMotorEx intakeMotor;
    //private final CRServo ContinuousServo;
    //private Servo Blocker;
    private final Command IntakeCommand;
//    private final Command Block;
//    private final Command Unblock;
//    private final Command Passthrough;


    // Constructor: pass in hardwareMap
    public Intake(HardwareMap hardwareMap) {
        // Initialize motor first
        intakeMotor = hardwareMap.get(DcMotorEx.class, "I");
        //Blocker = hardwareMap.get(Servo.class, "Blocker");
        //ContinuousServo = hardwareMap.get(CRServo.class, "CS");



//        Block = new LambdaCommand()
//                .setStart(() -> Blocker.setPosition(0.1)) // full power
//                .setInterruptible(true)
//                .named("Blocker Command");

        // LambdaCommand that runs until manually stopped
        IntakeCommand = new LambdaCommand()
                .setStart(() -> intakeMotor.setPower(1)) // full power
                .setUpdate(() -> intakeMotor.setPower(1)) // keep full power
                .setStop(interrupted -> intakeMotor.setPower(0)) // stop motor
                .setIsDone(() -> false) // never finishes on its own
                .setInterruptible(true)
                .named("Intake Command");

//        Unblock = new LambdaCommand()
//                .setStart(() -> Blocker.setPosition(1)) // full power
//                .setStop(interrupted -> Blocker.setPosition(1))
//                .setInterruptible(true)
//                .endAfter(2)
//                .named("Blocker Command");
//        Passthrough = new LambdaCommand()
//                .setStart(() -> ContinuousServo.setPower(1))
//                .setUpdate(() -> ContinuousServo.setPower(1))
//                .setStop(interrupted -> ContinuousServo.setPower(0))
//                .setIsDone(() -> false)
//                .setInterruptible(true)
//                .named("Passthrough Command");



    }

    // Schedule the command
    public void runIntake() {
        CommandManager.INSTANCE.scheduleCommand(IntakeCommand);
        IntakeCommand.start();
    }

    // Stop the shooter manually if needed
    public void stopIntake() {
        IntakeCommand.cancel();// stops the motor via setStop()
        //intakeMotor.setPower(0);
    }

//    public void BLOCK(){
//        //Block.schedule();
//        Blocker.setPosition(0.1);
//    }
//    public void UNBLOCK(){
//        Blocker.setPosition(1);
//    }

//    public Command Feed(){
//        return Passthrough;
//    }
//    public void StopFeed(){
//        Passthrough.cancel();
//    }

    public Command getCommand() {
        return IntakeCommand;
    }
//    public void testCS(){
//        ContinuousServo.setPower(1);
//    }
}
