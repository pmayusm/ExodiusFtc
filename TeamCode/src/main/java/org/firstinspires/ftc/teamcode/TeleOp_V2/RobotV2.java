package org.firstinspires.ftc.teamcode.TeleOp_V2;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotV2 {
    public MotorEx FrontLeft;
    public MotorEx FrontRight;
    public MotorEx BackLeft;
    public MotorEx BackRight;
    public MotorEx Shooter;










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

        Shooter = new MotorEx(hwMap, "SH", Motor.GoBILDA.RPM_1620);
        Shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);





    }
}



/*
config(10/17/2025):
0:BL
1:FL
2:BR
3:FR




*/
