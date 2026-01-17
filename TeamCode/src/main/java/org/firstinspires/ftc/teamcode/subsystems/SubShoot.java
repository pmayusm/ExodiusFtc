package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class SubShoot implements Subsystem {
    public static final SubShoot INSTANCE = new SubShoot();
    private SubShoot(){}

    private ServoEx HoodRot = new ServoEx("Hood");
    private MotorEx shooterMotor = new MotorEx("SH");
    private MotorEx shooterMotor2 = new MotorEx("SH2");
    private MotorGroup SHOOTERS = new MotorGroup(shooterMotor, shooterMotor2);
    public boolean PIDTRUE;
    double shottune;
    double hoodtune;

    private ControlSystem controlSystem = ControlSystem.builder()
            .velPid(0.1, 0, 0.01)
            .basicFF(0.004, 0.3, 0)
            .build();



    public Command hood1 = new SetPosition(HoodRot, 0).requires(this);
    public Command HoldShoot = new SetPower(shooterMotor, 1).requires(this);
    public Command HoldShoot2 = new SetPower(shooterMotor2, 1).requires(this);
    public Command StopShoot = new SetPower(shooterMotor, 0).requires(this);
    public Command StopShoot2 = new SetPower(shooterMotor2, 0).requires(this);
    public Command ReverseShoot = new SetPower(shooterMotor, -1).requires(this);
    public Command ReverseShoot2 = new SetPower(shooterMotor2, -1).requires(this);
    public Command AutoCloseShoot = new SetPower(shooterMotor, 0.83).requires(this);
    public Command AutoCloseShoot2 = new SetPower(shooterMotor2, 0.98).requires(this);
    public Command PIDshot = new RunToVelocity(controlSystem, 1180, 30).requires(this);
    public Command PIDstop = new RunToVelocity(controlSystem, 0, 2000).requires(this);

    public Command InterpolationTuning(){
        return new RunToVelocity(controlSystem, shottune, 30 ).requires(this);
    }
    public Command HoodInterpolation(){
        return new SetPosition(HoodRot, hoodtune).requires(this);
    }

    public double getvel(){
        return SHOOTERS.getVelocity();
    }
    public void setTargetvelocity(double targvel){

        shottune = targvel;
    }
    public double getTargetvelocity(){

        return shottune;
    }
    public void sethoodtune(double tunevalue){
        hoodtune = tunevalue;
    }
    public double getHoodtune(){
        return hoodtune;
    }





    @Override
    public void initialize() {
        HoodRot.setPosition(0.35);
        // initialization logic (runs on init)

        //shooterMotor.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void periodic() {
        // periodic logic (runs every loop)
        if (PIDTRUE){
            SHOOTERS.setPower(controlSystem.calculate(SHOOTERS.getState()));
            //shooterMotor.setPower(controlSystem.calculate(shooterMotor.getState()));
        }
        if (!PIDTRUE){
            SHOOTERS.setPower(0);
        }


    }
    public void setPIDTRUE(boolean pidstate){
        PIDTRUE = pidstate;
    }
    public boolean getPIDTRUE(){
        return PIDTRUE;
    }
}