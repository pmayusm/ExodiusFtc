package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

/**
 * Turret PID Subsystem for NextFTC Framework
 * Provides PID-controlled turret aiming for autonomous use
 */
public class TurretPIDSubsystem implements Subsystem {
    public static final TurretPIDSubsystem INSTANCE = new TurretPIDSubsystem();

    private TurretPIDSubsystem() {}

    // Motor
    private final MotorEx turretMotor = new MotorEx("TE").zeroed().brakeMode().reversed();

    // PID Constants
    private static final double TURRET_POWER_MAX = 1.0;
    private static final double TURRET_POWER_MIN = 0.3;
    private static final double Kp = 0.040;
    private static final double Ki = 0.001;
    private static final double Kd = 0.001;
    private static final double TARGET_TOLERANCE = 1.5;

    // PID State
    private double integralSum = 0;
    private double lastError = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    // Limelight subsystem reference
    private LimelightSubsystem limelight;

    /**
     * Initialize the subsystem with a limelight reference
     */
    @Override
    public void initialize() {
        pidTimer.reset();
    }

    /**
     * Set the limelight subsystem reference
     */
    public void setLimelight(LimelightSubsystem limelight) {
        this.limelight = limelight;
    }

    /**
     * Calculate PID output for turret aiming
     */
    private double calculatePID(double error) {
        // Proportional term
        double P = Kp * error;

        // Integral term
        integralSum += error * pidTimer.seconds();
        double I = Ki * integralSum;

        // Derivative term
        double D = Kd * (error - lastError) / pidTimer.seconds();

        pidTimer.reset();
        lastError = error;

        // Calculate total power
        double power = P + I + D;

        // Limit power to max values
        if (Math.abs(power) > TURRET_POWER_MAX) {
            power = Math.signum(power) * TURRET_POWER_MAX;
        } else if (Math.abs(error) > TARGET_TOLERANCE && Math.abs(power) < TURRET_POWER_MIN) {
            power = Math.signum(power) * TURRET_POWER_MIN;
        } else if (Math.abs(error) <= TARGET_TOLERANCE) {
            power = 0; // Stop when on target
        }

        return power;
    }

    /**
     * Reset PID state
     */
    public void resetPID() {
        integralSum = 0;
        lastError = 0;
        pidTimer.reset();
    }

    /**
     * Check if turret is on target
     */
    public boolean isOnTarget() {
        if (limelight == null || !limelight.isTargetFound()) {
            return false;
        }
        return Math.abs(limelight.getYawAngle()) < TARGET_TOLERANCE;
    }

    /**
     * Get current turret position
     */


    /**
     * Set turret power directly
     */
    public void setTurretPower(double power) {
        turretMotor.setPower(power);
    }

    /**
     * Stop turret
     */
    public void stopTurret() {
        turretMotor.setPower(0);
        resetPID();
    }

    /**
     * Command: Aim turret at AprilTag using PID
     * This command runs until the turret is on target
     */
    public Command aimAtTarget() {
        return new LambdaCommand()
                .requires(this)
                .setStart(() -> {
                    resetPID();
                    if (limelight != null) {
                        limelight.update();
                    }
                })
                .setUpdate(() -> {
                    if (limelight != null) {
                        limelight.update();

                        if (limelight.isTargetFound()) {
                            double yawError = limelight.getYawAngle();
                            double turretPower = calculatePID(yawError);
                            turretMotor.setPower(turretPower);
                        } else {
                            // No target found - stop moving
                            turretMotor.setPower(0);
                        }
                    }
                })
                .setStop(interrupted -> {
                    turretMotor.setPower(0);
                    resetPID();
                })
                .setIsDone(() -> isOnTarget())
                .named("Aim At Target");
    }

    /**
     * Command: Aim turret at target with timeout
     * @param timeoutSeconds Maximum time to spend aiming
     */
    public Command aimAtTargetWithTimeout(double timeoutSeconds) {
        return aimAtTarget().endAfter(timeoutSeconds);
    }

    /**
     * Command: Aim and hold position
     * This command continuously adjusts to maintain aim
     */
    public Command aimAndHold() {
        return new LambdaCommand()
                .requires(this)
                .setStart(() -> {
                    resetPID();
                    if (limelight != null) {
                        limelight.update();
                    }
                })
                .setUpdate(() -> {
                    if (limelight != null) {
                        limelight.update();

                        if (limelight.isTargetFound()) {
                            double yawError = limelight.getYawAngle();
                            double turretPower = calculatePID(yawError);
                            turretMotor.setPower(turretPower);
                        } else {
                            turretMotor.setPower(0);
                        }
                    }
                })
                .setStop(interrupted -> {
                    turretMotor.setPower(0);
                    resetPID();
                })
                .setIsDone(() -> false) // Never finishes on its own
                .setInterruptible(true)
                .named("Aim And Hold");
    }

    /**
     * Command: Stop turret
     */
    public Command stopTurretCommand() {
        return new LambdaCommand()
                .requires(this)
                .setStart(() -> stopTurret())
                .setIsDone(() -> true)
                .named("Stop Turret");
    }

    @Override
    public void periodic() {
        // Optional: Add telemetry or status updates here
    }
}