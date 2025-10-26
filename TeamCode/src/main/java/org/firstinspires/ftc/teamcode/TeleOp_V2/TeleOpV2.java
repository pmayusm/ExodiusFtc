package org.firstinspires.ftc.teamcode.TeleOp_V2;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name = "TeleOp V2")
public class TeleOpV2  extends LinearOpMode {
    static MecanumDrive drive;
    static GamepadEx gamepad1Ex;
    static GamepadEx gamepad2Ex;
    static RobotV2 robot;







//    private ColorSensor colorSensor;;
//    private double redValue;
//    private double greenValue;
//    private double blueValue;
//    private double alphaValue; //Light Intensity
//    private double TargetValue = 1000;
//private ColorSensor colorSensor;;
    private double redValue;
    private double greenValue;
    private double blueValue;
    private double alphaValue; //Light Intensity
    private double targetValue = 1000;

//    public void initColorSensor(){
//        colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");
//    }
//    public void getColor(){
//        redValue = colorSensor.red();
//        greenValue = colorSensor.green();
//        blueValue = colorSensor.blue();
//        alphaValue = colorSensor.alpha();
//    }
    public void colorTelemetry(){
        telemetry.addData("redValue",redValue);
        telemetry.addData("greenValue", greenValue);
        telemetry.addData("blueValue", blueValue);
        telemetry.addData("alphaValue", alphaValue);
        telemetry.update();
    }



    private void HardwareStart() {
        robot = new RobotV2();
        robot.init(hardwareMap);
        drive = new MecanumDrive(robot.FrontLeft, robot.FrontRight, robot.BackLeft, robot.BackRight);


        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);
        //initColorSensor();

        // Init Actions



    }
    //gamepad1, right bumber goes to close than open


    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime TimePassed = new ElapsedTime();
        int v_state = 0;

        waitForStart();
        HardwareStart();
        waitForStart();
        //getColor();
        //colorTelemetry();

        while (opModeIsActive()) {
            robot.Shooter.set(gamepad2Ex.getLeftY() * 0.7);
            drive.driveRobotCentric(
                    gamepad1Ex.getLeftX(),
                    gamepad1Ex.getLeftY(),
                    gamepad1Ex.getRightX(),
                    true
            );
            //getColor();
            //colorTelemetry();
//            switch (v_state)
//            {
//                case 0:
//                    robot.Coax.setPosition(1);
//                    TimePassed.reset();
//                    v_state++;
//                    break;
//                case 1:
//                    if (TimePassed.time() >= 2.0)
//                    {
//                        robot.IntakeClaw.setPosition(1);
//                        v_state = 0;
//                    }
//            }
//           STATE MACHINE FOR WAITS ABOVE
//           COLOR SENSOR BELOW
//            if(alphaValue > targetValue){

//                run action
//            } else {
//                run other action
//            }

            //start tele here

            if (gamepad1Ex.getButton(GamepadKeys.Button.A))
            {
                robot.FrontLeft.set(1);
            };




           // telemetry.addData("Extension Position Variable", extpos);
            telemetry.addData("TimePassed", TimePassed.time());
            telemetry.update();


        }
    }
}



