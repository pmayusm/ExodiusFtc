package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.positionable.SetPositions;

public class Shooter {

    private final DcMotorEx shooterMotor;
    private Servo controlservo;
    private Servo expansionservo;
    private final Command myLambdaCommand;
    //private final Command Shooterdefault;


    // Constructor: pass in hardwareMap
    public Shooter(HardwareMap hardwareMap) {
        // Initialize motor first
        controlservo = hardwareMap.get(Servo.class, "CS");
        expansionservo = hardwareMap.get(Servo.class, "ES");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "SH");

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
        //CommandManager.INSTANCE.scheduleCommand(myLambdaCommand);
        shooterMotor.setPower(1);
        //myLambdaCommand.start();
    }

    // Stop the shooter manually if needed
    public void stopShooter() {
        //myLambdaCommand.cancel();
        shooterMotor.setPower(0);// stops the motor via setStop()
    }

    public void defaultrot(){
        controlservo.setPosition(0.52);
        expansionservo.setPosition(0.48);
    }
    public void rot1(){
        controlservo.setPosition(0.63);
        expansionservo.setPosition(0.37);
    }
    public void setpos(double position){
        controlservo.setPosition(position);
        expansionservo.setPosition(1- position);
    }

    public void reverseShooter(){
        shooterMotor.setPower(1);
    }

    public Command getCommand() {
        return myLambdaCommand;
    }
}
