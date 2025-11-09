package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.VisionPortal;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

/**
 * Limelight subsystem for detecting AprilTag ID 20 and tracking pitch angle
 * Can be used in any OpMode by creating an instance and calling update()
 */
public class LimelightSubsystem {


    private Limelight3A limelight;
    private double pitchAngle = 0.0;
    private double yawAngle = 0.0;
    private boolean targetFound = false;
    private int targetAprilTagId = 20; // Default to ID 20
    private double botposeX;
    private double botposeZ;
    private double botYaw;

    private int targetAprilTagId2 = 24;

    /**
     * Constructor - initializes the Limelight
     * @param hardwareMap The OpMode's hardwareMap
     */
    public LimelightSubsystem(HardwareMap hardwareMap) {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(9); // Switch to pipeline 9 for AprilTag detection
        limelight.start();


    }

    /**
     * Constructor with custom AprilTag ID
     * @param hardwareMap The OpMode's hardwareMap
     * @param aprilTagId The AprilTag ID to track
     */
    public LimelightSubsystem(HardwareMap hardwareMap, int aprilTagId) {
        this(hardwareMap);
        this.targetAprilTagId = aprilTagId;
    }



    // Updates the pitch and yaw angles - MUST be called in OpMode loop

    public void update() {
        LLResult result = limelight.getLatestResult();
        Pose3D botPose = result.getBotpose();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            // Search for the target AprilTag ID
            for (LLResultTypes.FiducialResult fiducial : fiducialResults) {
                if (fiducial.getFiducialId() == targetAprilTagId || fiducial.getFiducialId() == 21 || fiducial.getFiducialId() == 22 || fiducial.getFiducialId() == 23 ) {
                    pitchAngle = fiducial.getTargetYDegrees();
                    yawAngle = fiducial.getTargetXDegrees();
                    botposeX = botPose.getPosition().x;
                    botposeZ = botPose.getPosition().z;
                    botYaw = botPose.getOrientation().getYaw();

                    targetFound = true;

                    return;
                }
            }
        }

        // No target found
        targetFound = false;
    }

    /**
     * Gets the pitch angle to the target AprilTag
     * @return The pitch angle in degrees
     */
    public double getPitchAngle() {
        return pitchAngle + 46;
    }
    public double getBotposeX(){
        return botposeX;
    }
    public double getBotposeZ(){
        return botposeZ;
    }
    public double getBotYaw(){
        return botYaw;
    }


    /**
     * Gets the yaw angle to the target AprilTag
     * @return The yaw angle in degrees
     */
    public double getYawAngle() {
        return yawAngle;
    }

    /**
     * Checks if the target AprilTag is currently detected
     * @return true if target is found, false otherwise
     */
    public boolean isTargetFound() {
        return targetFound;
    }


    /**
     * Sets the target AprilTag ID to track
     * @param id The AprilTag ID
     */
    public void setTargetAprilTagId(int id) {
        this.targetAprilTagId = id;
    }

    /**
     * Gets the current target AprilTag ID
     * @return The AprilTag ID being tracked
     */
    public int getTargetAprilTagId() {
        return targetAprilTagId;
    }

    /**
     * Estimates distance to the target based on pitch angle
     * Requires proper camera mounting measurements
     * @param cameraHeightInches Height of camera from ground
     * @param targetHeightInches Height of AprilTag from ground
     * @param cameraMountAngleDegrees Angle of camera mount (positive = angled up)
     * @return Estimated horizontal distance in inches
     */

    /**
     * Switches to a different Limelight pipeline
     * @param pipelineIndex The pipeline index (0-9)
     */
    public void switchPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    /**
     * Shuts down the Limelight - call in OpMode stop()
     */
    public void shutdown() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    /**
     * Gets the full Limelight result for advanced use
     * @return The latest LLResult
     */
    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

}