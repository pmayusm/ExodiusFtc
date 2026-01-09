package org.firstinspires.ftc.teamcode.subsystems;


import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;


public class SubIntake implements Subsystem {
    public static final SubIntake INSTANCE = new SubIntake();
    private SubIntake(){}
    private MotorEx IntakeMotor = new MotorEx("I");
    private ServoEx Blocker = new ServoEx("Kick");

    public LambdaCommand Intake = new LambdaCommand().requires(this)
            .setStart(() -> {
                IntakeMotor.setPower(1);
            }) // full power
            .setUpdate(() -> {
                IntakeMotor.setPower(1);
            }) // keep full power
            .setStop(interrupted ->  {
                IntakeMotor.setPower(0);
            }) // stop motor
            .setIsDone(() -> false) // never finishes on its own
            .setInterruptible(true)
            .named("Shooter Command");

    public Command KickUp = new SetPosition(Blocker, 0.97).requires(this);
    public Command KickDown = new SetPosition(Blocker, 0.45).requires(this);
    // Kickdown - opens blocker
    // KickUp - closes blocker
    public Command KickMiddle = new SetPosition(Blocker, 0.45).requires(this);
    public Command HoldIntake = new SetPower(IntakeMotor, 1).requires(this);
    public Command transferIntake = new SetPower(IntakeMotor, 0.6).requires(this);
    public Command StopIntake = new SetPower(IntakeMotor, 0).requires(this);
    public Command ReverseIntake = new SetPower(IntakeMotor, -1).requires(this);

    @Override
    public void periodic(){

    }

}