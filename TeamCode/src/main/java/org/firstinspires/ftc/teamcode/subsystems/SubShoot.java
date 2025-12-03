package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class SubShoot implements Subsystem {
    public static final SubShoot INSTANCE = new SubShoot();
    private SubShoot(){}

    private ServoEx HoodRot = new ServoEx("Hood");
    private MotorEx shooterMotor = new MotorEx("SH").zeroed();
    private MotorEx shooterMotor2 = new MotorEx("SH2").reversed();



    public LambdaCommand Shoot = new LambdaCommand().requires(this)
            .setStart(() -> {
                shooterMotor.setPower(1);
                shooterMotor2.setPower(1);
            }) // full power
            .setUpdate(() -> {
                shooterMotor.setPower(1);
                shooterMotor2.setPower(1);
            }) // keep full power
            .setStop(interrupted ->  {
                shooterMotor.setPower(0);
                shooterMotor2.setPower(0);
            }) // stop motor
            .setIsDone(() -> false) // never finishes on its own
            .setInterruptible(true)
            .named("Shooter Command");
    public Command hood1 = new SetPosition(HoodRot, 0).requires(this);
    public Command HoldShoot = new SetPower(shooterMotor, 1).requires(this);
    public Command HoldShoot2 = new SetPower(shooterMotor2, 1).requires(this);
    public Command StopShoot = new SetPower(shooterMotor, 0).requires(this);
    public Command StopShoot2 = new SetPower(shooterMotor2, 0).requires(this);
    public Command ReverseShoot = new SetPower(shooterMotor, -1).requires(this);
    public Command ReverseShoot2 = new SetPower(shooterMotor2, -1).requires(this);
    public Command AutoCloseShoot = new SetPower(shooterMotor, 0.98).requires(this);
    public Command AutoCloseShoot2 = new SetPower(shooterMotor2, 0.98).requires(this);

    public double getvel(){
        return shooterMotor.getVelocity();

    }

    //public Command Launch = new SetPower(shooterMotor, 1).requires(this);


    @Override
    public void initialize() {
        // initialization logic (runs on init)
        shooterMotor.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    @Override
    public void periodic() {
        // periodic logic (runs every loop)
    }
}
