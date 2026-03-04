package org.firstinspires.ftc.teamcode.TeleOp_V2;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SubIntake;
import org.firstinspires.ftc.teamcode.subsystems.SubShoot;
import org.firstinspires.ftc.teamcode.subsystems.SubTurret;

import java.util.concurrent.TimeUnit;

import dev.nextftc.control.ControlSystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Distance;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;


@TeleOp(name = "Telescoping Test")
public class TelescopingTest extends NextFTCOpMode {
    private MotorEx boxTube = new MotorEx("BT");
    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.005, 0, 0)
            .basicFF(0.000004, 0, 0)
            .build();


    public TelescopingTest() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    public Command extend = new RunToPosition(controlSystem, 1700, 30).requires(this);
    public Command Hold = new RunToVelocity(controlSystem, 0, 30).requires(this);

    //384.5 ticks per revolution
    // MAXIMUM THEORETICAL: 2787
    // applicable max: 2300
    @Override
    public void onInit(){

    }
    @Override
    public void onStartButtonPressed(){

    }

    @Override
    public void onUpdate(){
        boxTube.setPower(controlSystem.calculate(boxTube.getState()));
        telemetry.addData("Velocity", boxTube.getVelocity());
        if (gamepad1.a){
            extend.schedule();
        } else {
            Hold.schedule();
        }

//        if (gamepad1.a){
//            boxTube.setPower(-1);
//        }
//        if (gamepad1.b){
//            boxTube.setPower(1);
//        }
//        else {
//            boxTube.setPower(0);
//        }
    }
}
