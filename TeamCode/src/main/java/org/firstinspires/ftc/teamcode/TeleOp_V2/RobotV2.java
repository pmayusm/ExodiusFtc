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





    }
}

// Control Hub
// Motors:
//  0: FL - Front Left
//  1: BL - Back Left
//  2: LiftLeft
// Servo:
//  0: ExtL - Extension Left
//  1: ICoax - Intake Coaxial 4 Bar
//  2: IRot - Intake Rotation
//  3: OLW - Outtake Left Wrist
//  4: OLA - Outtake Left Arm
//  5: OCLaw - Outtake Claw

// Expansion Hub
// Motors:
//  0: FR - Front Right
//  1: BR - Back Right
//  2: LiftRight
// Servo:
//  0: ORA - Outtake Right Arm
//  1: ORot - Outtake Rotation
//  2: ORW - Outtake Right Wrist
//  3: IClaw - Intake Claw
//  4: IV4B - Intake Virtual 4 Bar
//  5: ExtR - Extension Right

/* NEW CONFIG(regionals)
CH:
    Motors:
        0: FR
        1: BR
        2: FL
        3: Bl
    Servos:
        0:OLA
        1:Orot
        2:Oclaw
        3:OLW
        4:ORW
        5:ORA
EH:
    Motors:
        2:LiftRight
        3:LiftLeft
    Servos:
        0:ICoax
        1:Iclaw
        2:ExtL
        3:ExtR
        4:IV4B
        5:Irot



*/
