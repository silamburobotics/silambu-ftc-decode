package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

/*
 * This OpMode demonstrates autonomous ball detection and pursuit using computer vision
 * and Road Runner for smooth trajectory following.
 * 
 * The robot will:
 * 1. Scan for purple or green balls using color detection
 * 2. Calculate the position of the nearest ball
 * 3. Drive towards it using Road Runner trajectories
 * 4. Stop when close enough or ball is lost
 *
 * Requirements:
 * - Camera configured as "Webcam 1" in robot configuration
 * - MecanumDrive properly tuned
 * - Purple or green colored balls in the field
 */

@Autonomous(name = "Ball Chase Auto", group = "Vision")
public class BallChaseAutonomous extends LinearOpMode {
    
    // Vision components
    private VisionPortal visionPortal;
    private PredominantColorProcessor colorProcessor;
    
    // Drive system
    private MecanumDrive drive;
    
    // Ball detection parameters
    private static final double BALL_APPROACH_DISTANCE = 12.0; // inches from ball to stop
    private static final double MAX_DRIVE_SPEED = 0.6;
    private static final double DETECTION_TIMEOUT = 30.0; // seconds
    
    // Camera and field parameters
    private static final double CAMERA_HEIGHT = 8.0; // inches above ground
    private static final double CAMERA_PITCH = -15.0; // degrees (negative = pointing down)
    private static final double BALL_HEIGHT = 3.0; // inches (typical ball diameter)
    
    @Override
    public void runOpMode() {
        // Initialize drive system
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        
        // Initialize vision system
        initializeVision();
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Vision", "Looking for purple/green balls");
        telemetry.addData("Controls", "Start to begin ball chase");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            double startTime = getRuntime();
            
            while (opModeIsActive() && (getRuntime() - startTime) < DETECTION_TIMEOUT) {
                BallDetectionResult ball = detectNearestBall();
                
                if (ball != null) {
                    telemetry.addData("Ball Found", ball.color);
                    telemetry.addData("Distance", "%.1f inches", ball.estimatedDistance);
                    telemetry.addData("Angle", "%.1f degrees", ball.angleFromCenter);
                    
                    if (ball.estimatedDistance > BALL_APPROACH_DISTANCE) {
                        driveTowardsBall(ball);
                    } else {
                        telemetry.addData("Status", "Reached ball!");
                        break;
                    }
                } else {
                    telemetry.addData("Status", "Searching for ball...");
                    // Slow rotation to search for balls
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0.2));
                }
                
                telemetry.update();
                sleep(100);
            }
            
            // Stop the robot
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            telemetry.addData("Status", "Ball chase complete");
            telemetry.update();
        }
        
        // Cleanup
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
    
    private void initializeVision() {
        // Configure color processor to detect purple and green
        colorProcessor = new PredominantColorProcessor.Builder()
            .setRoi(ImageRegion.entireFrame()) // Use full camera view
            .setSwatches(
                PredominantColorProcessor.Swatch.PURPLE,
                PredominantColorProcessor.Swatch.GREEN,
                // Add artifact colors for better FTC game piece detection
                PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                PredominantColorProcessor.Swatch.ARTIFACT_GREEN
            )
            .build();
        
        // Create vision portal with optimized settings
        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessor(colorProcessor)
            .setCameraResolution(new Size(640, 480))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .enableLiveView(true)
            .build();
    }
    
    private BallDetectionResult detectNearestBall() {
        PredominantColorProcessor.Result result = colorProcessor.getAnalysis();
        
        // Check if we detected a purple or green color
        if (result.closestSwatch == PredominantColorProcessor.Swatch.PURPLE ||
            result.closestSwatch == PredominantColorProcessor.Swatch.GREEN ||
            result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE ||
            result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN) {
            
            return new BallDetectionResult(
                result.closestSwatch.toString(),
                estimateDistanceToFloor(result),
                calculateAngleFromCenter(result)
            );
        }
        
        return null;
    }
    
    private double estimateDistanceToFloor(PredominantColorProcessor.Result result) {
        // Simple distance estimation based on color region size and camera geometry
        // This is a rough approximation - more sophisticated methods could use stereo vision
        // or known object sizes for better accuracy
        
        // For now, use a simple heuristic based on the predominant color area
        // Larger color regions = closer objects
        int[] hsv = result.HSV;
        double saturation = hsv[1]; // Higher saturation often means closer/more prominent object
        
        // Simple linear mapping - adjust these values based on your camera setup
        double baseDistance = 36.0; // inches
        double saturationFactor = (255.0 - saturation) / 255.0;
        
        return baseDistance * (0.5 + saturationFactor);
    }
    
    private double calculateAngleFromCenter(PredominantColorProcessor.Result result) {
        // Calculate horizontal angle based on where the color is detected
        // This is simplified - a more accurate method would track the centroid of the detected region
        
        // For now, assume the predominant color is roughly centered
        // In a more advanced implementation, you'd track the actual position of the color blob
        return 0.0; // Straight ahead - implement blob tracking for actual angle calculation
    }
    
    private void driveTowardsBall(BallDetectionResult ball) {
        // Calculate target position based on ball location
        double forwardDistance = Math.max(ball.estimatedDistance - BALL_APPROACH_DISTANCE, 6.0);
        double angleRadians = Math.toRadians(ball.angleFromCenter);
        
        // Calculate target position
        Vector2d currentPos = drive.localizer.getPose().position;
        Vector2d targetOffset = new Vector2d(
            forwardDistance * Math.cos(angleRadians),
            forwardDistance * Math.sin(angleRadians)
        );
        Vector2d targetPos = currentPos.plus(targetOffset);
        
        // Create and execute trajectory
        TrajectoryActionBuilder trajectoryBuilder = drive.actionBuilder(drive.localizer.getPose())
            .strafeToLinearHeading(targetPos, drive.localizer.getPose().heading.toDouble());
        
        // Execute movement with speed limiting
        Actions.runBlocking(trajectoryBuilder.build());
    }
    
    /**
     * Data class to hold ball detection results
     */
    private static class BallDetectionResult {
        public final String color;
        public final double estimatedDistance;
        public final double angleFromCenter;
        
        public BallDetectionResult(String color, double distance, double angle) {
            this.color = color;
            this.estimatedDistance = distance;
            this.angleFromCenter = angle;
        }
    }
}