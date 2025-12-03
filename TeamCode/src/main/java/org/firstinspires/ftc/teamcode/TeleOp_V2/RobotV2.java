package org.firstinspires.ftc.teamcode.TeleOp_V2;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotV2 {
    public MotorEx FrontLeft;
    public MotorEx FrontRight;
    public MotorEx BackLeft;
    public MotorEx BackRight;
//    public MotorEx TurretSpin;
    public DcMotor TurretEnc;
    public MotorEx SH;
    public MotorEx SH2;

    double ticks = 384.5;
    double newTarget;










    private static HardwareMap hwMapRobot;


    public void init(HardwareMap hwMap) {
        hwMapRobot = hwMap;



        FrontLeft = new MotorEx(hwMap, "FL", Motor.GoBILDA.RPM_435);
        FrontLeft.setInverted(true);
        FrontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        FrontRight = new MotorEx(hwMap, "FR", Motor.GoBILDA.RPM_435);
        FrontRight.setInverted(true);
        FrontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        BackLeft = new MotorEx(hwMap, "BL", Motor.GoBILDA.RPM_435);
        BackLeft.setInverted(true);
        BackLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        BackRight = new MotorEx(hwMap, "BR", Motor.GoBILDA.RPM_435);
        BackRight.setInverted(true);
        BackRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


//        TurretSpin = new MotorEx(hwMap, "TS", Motor.GoBILDA.RPM_435);
//        TurretSpin.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        TurretSpin.setRunMode(Motor.RunMode.PositionControl);

        TurretEnc = hwMap.get(DcMotor.class, "TE");
        TurretEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TurretEnc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TurretEnc.setDirection(DcMotorSimple.Direction.REVERSE);

        SH = new MotorEx(hwMap, "SH", Motor.GoBILDA.RPM_1620);

        SH2 = new MotorEx(hwMap, "SH2", Motor.GoBILDA.RPM_1620);
        SH2.setInverted(true);







    }
    public void TurretEncoderReset(){
        TurretEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void untangle(int targetticks){
        TurretEnc.setTargetPosition(targetticks);
        TurretEnc.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
//    public void turnToAprilTag(double yawerror){
//        double turretDegreesNeeded = yawerror;
//        double motorRevolutionsNeeded = (turretDegreesNeeded/360.0) * 3.1;  //3.1 is the gear ratio from motor to turret (1:3.1)
//        double ticksNeeded = motorRevolutionsNeeded * ticks;
//        double CurrentPosition = TurretEnc.getCurrentPosition();
//        double newTarget = CurrentPosition + ticksNeeded;
//        TurretEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        TurretEnc.setTargetPosition((int)newTarget);
//        TurretEnc.setPower(0.3);
//        TurretEnc.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//    }
}



/*
config(10/17/2025):
0:BL
1:FL
2:BR
3:FR




*/
