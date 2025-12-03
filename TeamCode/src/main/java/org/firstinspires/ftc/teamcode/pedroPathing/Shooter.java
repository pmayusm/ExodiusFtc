package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.MotorPIDVelocity;

import java.util.Set;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.positionable.SetPositions;
import dev.nextftc.hardware.powerable.SetPower;

public class Shooter {

    private final DcMotorEx shooterMotor;
    private final DcMotorEx shooterMotor2;
    //MotorEx shooterMotor = new MotorEx("SH");
    private final double MAX_VELOCITY_TPS = 2450; // Tune this value based on your tests
    MotorPIDVelocity pidController = new MotorPIDVelocity(0, 0, 0); // Tune these Kp, Ki, Kd values
    //private Servo controlservo;
    private Servo expansionservo;
    //MotorEx turretSpin = new MotorEx("TS");
    //private final DcMotorEx turretSpin;
    private final Command Shoot;










    // Constructor: pass in hardwareMap
    public Shooter(HardwareMap hardwareMap) {
        // Initialize motor first
        //controlservo = hardwareMap.get(Servo.class, "CS");
        expansionservo = hardwareMap.get(Servo.class, "ES");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "SH");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "SH2");




        // LambdaCommand that runs until manually stopped
        Shoot = new LambdaCommand()
                .setStart(() -> {
                    shooterMotor.setPower(1);
                    shooterMotor2.setPower(-1);
                }) // full power
                .setUpdate(() -> {
                    shooterMotor.setPower(1);
                    shooterMotor2.setPower(-1);
                }) // keep full power
                .setStop(interrupted ->  {
                    shooterMotor.setPower(0);
                    shooterMotor2.setPower(0);
                }) // stop motor
                .setIsDone(() -> false) // never finishes on its own
                .setInterruptible(true)
                .named("Shooter Command");











    }

    // Schedule the command
    public void runShooter() {

        //CommandManager.INSTANCE.scheduleCommand(myLambdaCommand);
        CommandManager.INSTANCE.scheduleCommand(Shoot);
        //shooterMotor.setPower(1);
        //myLambdaCommand.start();
    }

    // Stop the shooter manually if needed
    public void stopShooter() {
        //myLambdaCommand.cancel();// stops the motor via setStop()
        Shoot.cancel();
        //shooterMotor.setPower(0);
    }

    public void defaultrot(){
        //controlservo.setPosition(0.52);
        expansionservo.setPosition(0.48);
    }
    public void rot1(){
        //controlservo.setPosition(0.63);
        expansionservo.setPosition(0.37);
    }
    public void setpos(double position){
        //controlservo.setPosition(position);
        expansionservo.setPosition(position);
    }
//    public void reverseShooter(){
//        shooterMotor.setPower(1);
//    }


    public Command getCommand() {
        return Shoot;
    }
}
