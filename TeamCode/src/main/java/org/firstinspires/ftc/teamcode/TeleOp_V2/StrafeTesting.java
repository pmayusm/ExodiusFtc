package org.firstinspires.ftc.teamcode.TeleOp_V2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class StrafeTesting extends LinearOpMode {
    public DcMotorEx FrontLeft;
    public DcMotorEx FrontRight;
    public DcMotorEx BackLeft;
    public DcMotorEx BackRight;

    @Override
    public void runOpMode() throws InterruptedException {
        FrontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontRight = hardwareMap.get(DcMotorEx.class, "FR");

        BackRight = hardwareMap.get(DcMotorEx.class, "BR");

        BackLeft = hardwareMap.get(DcMotorEx.class, "BL");
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            FrontLeft.setPower(-0.5);
            BackRight.setPower(-0.55);
            FrontRight.setPower(0.5);
            BackLeft.setPower(0.55);
        }
    }
}