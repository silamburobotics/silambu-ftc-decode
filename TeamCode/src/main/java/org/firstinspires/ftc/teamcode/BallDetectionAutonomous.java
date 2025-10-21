package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

/*
 * Autonomous OpMode that detects the nearest purple or green ball
 * and drives the robot close to it using Road Runner trajectories
 * 
 * This OpMode will:
 * 1. Initialize the camera and color detection system
 * 2. Scan for purple or green colored objects (balls)
 * 3. Calculate the position and distance to the nearest ball
 * 4. Use Road Runner to drive smoothly towards the ball
 * 5. Stop at a safe distance from the target
 * 
 * Hardware Requirements:
 * - Camera configured as "Webcam 1"
 * - MecanumDrive with properly configured motors and IMU
 * - Purple or green colored balls in the field of view
 */

@Autonomous(name = "Ball Detection Auto", group = "Autonomous")
public class BallDetectionAutonomous extends LinearOpMode {
    
    // Vision system components
    private VisionPortal visionPortal;
    private PredominantColorProcessor colorProcessor;
    
    // Drive system
    private MecanumDrive drive;
    
    // Timing and control
    private ElapsedTime runtime = new ElapsedTime();
    
    // Configuration constants
    private static final double APPROACH_DISTANCE_INCHES = 8.0;  // How close to get to the ball
    private static final double MAX_SEARCH_TIME_SECONDS = 30.0;  // Maximum time to search for ball
    private static final double SEARCH_ROTATION_SPEED = 0.3;     // Speed when searching
    private static final double APPROACH_SPEED = 0.5;           // Speed when approaching ball
    private static final double MIN_CONFIDENCE_THRESHOLD = 100;  // Minimum color saturation for detection
    
    // Camera configuration
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;
    private static final double CAMERA_FOV_DEGREES = 60.0;      // Horizontal field of view
    
    @Override
    public void runOpMode() {
        // Initialize the drive system
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        
        // Initialize vision system
        initializeVision();
        
        // Display initialization status
        telemetry.addData("Status", "Initialized and Ready");
        telemetry.addData("Mission", "Find nearest purple/green ball");
        telemetry.addData("Camera", "Looking for targets...");
        telemetry.addLine();
        telemetry.addData("Approach Distance", "%.1f inches", APPROACH_DISTANCE_INCHES);
        telemetry.addData("Max Search Time", "%.1f seconds", MAX_SEARCH_TIME_SECONDS);
        telemetry.update();
        
        // Wait for start
        waitForStart();
        runtime.reset();
        
        if (opModeIsActive()) {
            executeAutonomousMission();
        }
        
        // Cleanup
        shutdownSystems();
    }
    
    /**
     * Initialize the vision system with color detection
     */
    private void initializeVision() {
        // Configure color processor to detect purple and green balls
        colorProcessor = new PredominantColorProcessor.Builder()
            // Use center region of camera for detection
            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.4, 0.4, 0.4, -0.4))
            // Define target colors including FTC game piece colors
            .setSwatches(
                PredominantColorProcessor.Swatch.PURPLE,
                PredominantColorProcessor.Swatch.GREEN,
                PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                PredominantColorProcessor.Swatch.ARTIFACT_GREEN
            )
            .build();
        
        // Create vision portal with optimized settings
        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessor(colorProcessor)
            .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .enableLiveView(true)  // Enable camera preview on Driver Station
            .build();
        
        // Wait for camera to be ready
        while (!visionPortal.getCameraState().equals(VisionPortal.CameraState.STREAMING) && !isStopRequested()) {
            telemetry.addData("Camera", "Initializing...");
            telemetry.update();
            sleep(50);
        }
        
        telemetry.addData("Camera", "Ready");
        telemetry.update();
    }
    
    /**
     * Main autonomous mission execution
     */
    private void executeAutonomousMission() {
        BallDetectionResult targetBall = null;
        boolean missionComplete = false;
        
        telemetry.addData("Mission Status", "SEARCHING for ball");
        telemetry.update();
        
        // Main mission loop
        while (opModeIsActive() && !missionComplete && runtime.seconds() < MAX_SEARCH_TIME_SECONDS) {
            
            // Scan for target ball
            targetBall = scanForBall();
            
            if (targetBall != null) {
                telemetry.addData("Mission Status", "BALL DETECTED");
                telemetry.addData("Ball Color", targetBall.color);
                telemetry.addData("Estimated Distance", "%.1f inches", targetBall.estimatedDistance);
                telemetry.addData("Direction", "%.1f degrees", targetBall.direction);
                telemetry.update();
                
                // Check if we're close enough
                if (targetBall.estimatedDistance <= APPROACH_DISTANCE_INCHES) {
                    telemetry.addData("Mission Status", "TARGET REACHED!");
                    missionComplete = true;
                } else {
                    // Drive towards the ball
                    driveTowardsBall(targetBall);
                }
                
            } else {
                // No ball detected - search by rotating
                telemetry.addData("Mission Status", "SEARCHING...");
                telemetry.addData("Search Time", "%.1f / %.1f seconds", 
                    runtime.seconds(), MAX_SEARCH_TIME_SECONDS);
                telemetry.update();
                
                searchForBall();
            }
            
            sleep(100); // Small delay for processing
        }
        
        // Mission complete - stop robot
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        
        // Final status
        if (missionComplete) {
            telemetry.addData("Mission", "COMPLETED SUCCESSFULLY!");
            telemetry.addData("Final Distance", "%.1f inches", 
                targetBall != null ? targetBall.estimatedDistance : 0);
        } else {
            telemetry.addData("Mission", "TIMEOUT - No ball found");
        }
        
        telemetry.addData("Total Time", "%.1f seconds", runtime.seconds());
        telemetry.update();
        
        // Hold position
        while (opModeIsActive()) {
            sleep(100);
        }
    }
    
    /**
     * Scan for purple or green balls using color detection
     */
    private BallDetectionResult scanForBall() {
        PredominantColorProcessor.Result result = colorProcessor.getAnalysis();
        
        // Check if we detected a target color
        if (isTargetColor(result.closestSwatch)) {
            // Verify confidence by checking color saturation
            if (result.HSV != null && result.HSV[1] > MIN_CONFIDENCE_THRESHOLD) {
                
                double distance = estimateDistance(result);
                double direction = estimateDirection(result);
                
                return new BallDetectionResult(
                    result.closestSwatch.toString(),
                    distance,
                    direction,
                    result.HSV[1] // Use saturation as confidence measure
                );
            }
        }
        
        return null; // No valid ball detected
    }
    
    /**
     * Check if the detected color is one of our targets
     */
    private boolean isTargetColor(PredominantColorProcessor.Swatch swatch) {
        return swatch == PredominantColorProcessor.Swatch.PURPLE ||
               swatch == PredominantColorProcessor.Swatch.GREEN ||
               swatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE ||
               swatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN;
    }
    
    /**
     * Estimate distance to ball based on color properties
     */
    private double estimateDistance(PredominantColorProcessor.Result result) {
        // Simple distance estimation based on color saturation and value
        // Higher saturation and value typically indicate closer objects
        double saturation = result.HSV[1];
        double value = result.HSV[2];
        
        // Empirical formula - adjust based on your camera and lighting
        double baseDistance = 36.0; // Base distance in inches
        double saturationFactor = (255.0 - saturation) / 255.0;
        double valueFactor = (255.0 - value) / 255.0;
        
        return baseDistance * (0.3 + 0.4 * saturationFactor + 0.3 * valueFactor);
    }
    
    /**
     * Estimate direction to ball (simplified - assumes ball is centered)
     */
    private double estimateDirection(PredominantColorProcessor.Result result) {
        // For now, assume ball is roughly centered in the ROI
        // In a more advanced implementation, you would track the actual centroid
        return 0.0; // Straight ahead
    }
    
    /**
     * Drive towards the detected ball using Road Runner
     */
    private void driveTowardsBall(BallDetectionResult ball) {
        // Calculate target position
        double moveDistance = Math.max(ball.estimatedDistance - APPROACH_DISTANCE_INCHES, 3.0);
        double angleRadians = Math.toRadians(ball.direction);
        
        // Calculate target in robot coordinate system
        Vector2d targetOffset = new Vector2d(
            moveDistance * Math.cos(angleRadians),
            moveDistance * Math.sin(angleRadians)
        );
        
        // Get current pose and calculate field target
        Pose2d currentPose = drive.localizer.getPose();
        Vector2d fieldTarget = currentPose.position.plus(targetOffset);
        
        // Create and execute trajectory
        TrajectoryActionBuilder trajectoryBuilder = drive.actionBuilder(currentPose)
            .strafeToLinearHeading(fieldTarget, currentPose.heading.toDouble());
        
        // Execute the trajectory
        Actions.runBlocking(trajectoryBuilder.build());
        
        telemetry.addData("Action", "Moving towards ball");
        telemetry.addData("Move Distance", "%.1f inches", moveDistance);
        telemetry.update();
    }
    
    /**
     * Search for ball by rotating slowly
     */
    private void searchForBall() {
        // Slow rotation to search for balls
        drive.setDrivePowers(new PoseVelocity2d(
            new Vector2d(0, 0), 
            SEARCH_ROTATION_SPEED
        ));
    }
    
    /**
     * Shutdown all systems
     */
    private void shutdownSystems() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
    
    /**
     * Data class to hold ball detection results
     */
    private static class BallDetectionResult {
        public final String color;
        public final double estimatedDistance;
        public final double direction;
        public final double confidence;
        
        public BallDetectionResult(String color, double distance, double direction, double confidence) {
            this.color = color;
            this.estimatedDistance = distance;
            this.direction = direction;
            this.confidence = confidence;
        }
    }
}