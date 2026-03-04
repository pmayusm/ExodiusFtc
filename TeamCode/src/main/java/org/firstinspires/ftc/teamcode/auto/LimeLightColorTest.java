package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.ColorResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

import dev.nextftc.hardware.impl.ServoEx;

@Autonomous
public class LimeLightColorTest extends OpMode {

    private Limelight3A limelight3A;
    Servo servo;


    @Override
    public void init(){
        servo = hardwareMap.get(Servo.class, "Light");
        servo.setPosition(1);
        telemetry.setMsTransmissionInterval(11);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(8); // 8 is color detection
        telemetry.addData("Status", "Initialized - Make sure 'Send Corners' is enabled in Output tab!");
        telemetry.update();
    }

    @Override
    public void start(){
        servo.setPosition(1);
        limelight3A.start();
    }

    @Override
    public void loop(){
        servo.setPosition(1);
        LLResult llResult = limelight3A.getLatestResult();
        LLStatus status = limelight3A.getStatus();

        if (llResult != null && llResult.isValid()){
            // Basic targeting data
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());

            // Get color results to access corners
            List<ColorResult> colorResults = llResult.getColorResults();

            if (colorResults != null && !colorResults.isEmpty()) {
                telemetry.addData("Targets Found", colorResults.size());

                // Loop through each detected color target
                for (int i = 0; i < colorResults.size(); i++) {
                    ColorResult target = colorResults.get(i);

                    // Get the corners for this target
                    // Returns List<List<Double>> where each inner list is [x, y]
                    List<List<Double>> corners = target.getTargetCorners();

                    if (corners != null && !corners.isEmpty()) {
                        telemetry.addData("Target " + i + " Corners", corners.size());

                        // Display each corner's coordinates
                        for (int j = 0; j < corners.size(); j++) {
                            List<Double> corner = corners.get(j);
                            if (corner != null && corner.size() >= 2) {
                                double x = corner.get(0); // X coordinate in pixels
                                double y = corner.get(1); // Y coordinate in pixels

                                telemetry.addData("  Corner " + j,
                                        String.format("(%.2f, %.2f)", x, y));
                            }
                        }
                    } else {
                        telemetry.addData("Target " + i, "No corners available");
                        telemetry.addData("Note", "Enable 'Send Corners' in Output tab");
                    }
                }
            } else {
                telemetry.addData("Color Results", "No targets detected");
            }

        } else {
            telemetry.addData("Limelight", "No valid results");
        }

        telemetry.update();
    }
}