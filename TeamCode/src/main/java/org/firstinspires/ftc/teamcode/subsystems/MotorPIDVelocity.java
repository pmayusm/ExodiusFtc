package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MotorPIDVelocity {

    // tune kp, kd, and ki
    private double kP, kI, kD;
    private double previousError = 0;
    private double integralSum = 0;
    private ElapsedTime timer = new ElapsedTime();

    public MotorPIDVelocity(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        timer.reset();
    }
    public double calculate(double targetSpeed, double currentSpeed){
        double error = targetSpeed - currentSpeed;
        double deltatime = timer.seconds();
        timer.reset();
        integralSum += error * deltatime;
        double derivative = (error - previousError)/deltatime;
        previousError = error;
        return (kP * error) + (kI * integralSum) + (kD * derivative);
    }

    // how to use :
//    while (opModeIsActive()) {
//        double currentMotorVelocity = motor.getVelocity(); // Get velocity in ticks/second
//        double motorPower = velocityPID.calculate(targetMotorSpeed, currentMotorVelocity);
//        motor.setPower(motorPower);
//    }
}
