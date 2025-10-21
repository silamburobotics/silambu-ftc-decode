package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

/*
 * TeleOp mode for testing ball detection and manual/assisted driving
 * 
 * Controls:
 * - Left stick: Manual drive (strafe)
 * - Right stick: Manual drive (rotate)
 * - A button: Enable ball tracking assist
 * - B button: Disable ball tracking assist
 * - Y button: Search mode (slow rotation)
 */

@TeleOp(name = "Ball Detection TeleOp", group = "Vision")
public class BallDetectionTeleOp extends LinearOpMode {
    
    private VisionPortal visionPortal;
    private PredominantColorProcessor colorProcessor;
    private MecanumDrive drive;
    
    private boolean ballTrackingEnabled = false;
    private boolean searchMode = false;
    
    // Drive parameters
    private static final double DRIVE_SPEED_MULTIPLIER = 0.8;
    private static final double TRACKING_SPEED = 0.4;
    private static final double SEARCH_SPEED = 0.2;
    
    @Override
    public void runOpMode() {
        // Initialize systems
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        initializeVision();
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "A=Track, B=Manual, Y=Search");
        telemetry.addLine("Left stick: strafe, Right stick: rotate");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            handleDriverControls();
            
            if (ballTrackingEnabled || searchMode) {
                handleBallTracking();
            }
            
            updateTelemetry();
            sleep(20);
        }
        
        // Cleanup
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
    
    private void initializeVision() {
        colorProcessor = new PredominantColorProcessor.Builder()
            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.3, 0.3, 0.3, -0.3))
            .setSwatches(
                PredominantColorProcessor.Swatch.PURPLE,
                PredominantColorProcessor.Swatch.GREEN,
                PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                PredominantColorProcessor.Swatch.BLACK // For no detection
            )
            .build();
        
        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessor(colorProcessor)
            .setCameraResolution(new Size(640, 480))
            .enableLiveView(true)
            .build();
    }
    
    private void handleDriverControls() {
        // Button controls
        if (gamepad1.a && !ballTrackingEnabled) {
            ballTrackingEnabled = true;
            searchMode = false;
            gamepad1.rumble(100); // Brief rumble feedback
        }
        
        if (gamepad1.b && ballTrackingEnabled) {
            ballTrackingEnabled = false;
            searchMode = false;
        }
        
        if (gamepad1.y) {
            searchMode = !searchMode;
            ballTrackingEnabled = false;
        }
        
        // Manual driving (only when not in tracking mode)
        if (!ballTrackingEnabled && !searchMode) {
            double strafe = -gamepad1.left_stick_x * DRIVE_SPEED_MULTIPLIER;
            double forward = -gamepad1.left_stick_y * DRIVE_SPEED_MULTIPLIER;
            double rotate = -gamepad1.right_stick_x * DRIVE_SPEED_MULTIPLIER;
            
            drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(forward, strafe), rotate));
        }
    }
    
    private void handleBallTracking() {
        PredominantColorProcessor.Result result = colorProcessor.getAnalysis();
        
        boolean ballDetected = (result.closestSwatch == PredominantColorProcessor.Swatch.PURPLE ||
                               result.closestSwatch == PredominantColorProcessor.Swatch.GREEN ||
                               result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE ||
                               result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN);
        
        if (ballTrackingEnabled && ballDetected) {
            // Ball detected - move towards it
            double forwardPower = TRACKING_SPEED;
            
            // Simple turning based on color saturation distribution
            // This is a simplified approach - the advanced version has better position tracking
            double turnPower = 0.0;
            
            // Apply gentle movement towards detected ball
            drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(forwardPower, 0), turnPower));
                
        } else if (searchMode) {
            // Search mode - slowly rotate to find balls
            drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(0, 0), SEARCH_SPEED));
                
        } else if (ballTrackingEnabled && !ballDetected) {
            // Ball tracking enabled but no ball found - stop
            drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(0, 0), 0));
        }
    }
    
    private void updateTelemetry() {
        PredominantColorProcessor.Result result = colorProcessor.getAnalysis();
        
        telemetry.addData("Mode", ballTrackingEnabled ? "TRACKING" : 
                                 searchMode ? "SEARCHING" : "MANUAL");
        telemetry.addData("Ball Detected", result.closestSwatch);
        
        if (result.RGB != null) {
            telemetry.addData("RGB", "(%d, %d, %d)", 
                result.RGB[0], result.RGB[1], result.RGB[2]);
        }
        
        if (result.HSV != null) {
            telemetry.addData("HSV", "(%d, %d, %d)", 
                result.HSV[0], result.HSV[1], result.HSV[2]);
        }
        
        // Control instructions
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addData("A Button", "Enable ball tracking");
        telemetry.addData("B Button", "Manual control");
        telemetry.addData("Y Button", "Toggle search mode");
        
        // Robot position
        Pose2d pose = drive.localizer.getPose();
        telemetry.addLine();
        telemetry.addData("Robot X", "%.1f", pose.position.x);
        telemetry.addData("Robot Y", "%.1f", pose.position.y);
        telemetry.addData("Robot Heading", "%.1f°", Math.toDegrees(pose.heading.toDouble()));
        
        telemetry.update();
    }
}