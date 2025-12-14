package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

public class SubTurret implements Subsystem {
    public static final SubTurret INSTANCE = new SubTurret();
    private SubTurret(){}
    private MotorEx TurretMotor = new MotorEx("TE").reversed();
    public double target;
    //145.1 ticks per revolution
    // 3.1 : 1 gear ratio
    private ControlSystem aimer = ControlSystem.builder()
            .posPid(0.009, 0 , 0.0002)
            .build();

    public Command TestRun = new RunToPosition(aimer, 0, 10).requires(this);
    public Command TestRun2 = new RunToPosition(aimer, 112.4525).requires(this);
    // turns 90 degrees to the right
    public Command TestRun3 = new RunToPosition(aimer, -21.7).requires(this);

    //public Command AIMER = new RunToPosition(aimer, target).requires(this);
    public Command AIMER(){
        return new RunToPosition(aimer,target).requires(this);
    };

    @Override
    public void initialize() {
        // initialization logic (runs on init)
        TurretMotor.zeroed();
        TurretMotor.zero();
    }
    @Override
    public void periodic() {
        // periodic logic (runs every loop)
        TurretMotor.setPower(aimer.calculate(TurretMotor.getState()));
    }
    public void setTarget(double turnage){
        target = turnage;
    }
    public double getTarget(){
        return target;
    }

}
