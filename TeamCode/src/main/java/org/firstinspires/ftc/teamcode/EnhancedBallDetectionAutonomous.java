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
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/*
 * Enhanced Autonomous OpMode with precise ball detection using OpenCV
 * 
 * This version provides:
 * - Accurate ball position tracking using blob detection
 * - Real distance estimation based on ball size
 * - Precise angle calculation for steering
 * - Multiple ball tracking with nearest selection
 * - Robust color filtering for purple and green balls
 */

@Autonomous(name = "Enhanced Ball Auto", group = "Autonomous")
public class EnhancedBallDetectionAutonomous extends LinearOpMode {
    
    // Hardware components
    private OpenCvCamera camera;
    private BallTrackingPipeline pipeline;
    private MecanumDrive drive;
    
    // Timing
    private ElapsedTime runtime = new ElapsedTime();
    
    // Mission parameters
    private static final double TARGET_DISTANCE_INCHES = 6.0;   // How close to approach
    private static final double MAX_MISSION_TIME = 30.0;        // Maximum autonomous time
    private static final double SEARCH_TIMEOUT = 5.0;          // Time to search before moving
    
    // Camera and field parameters
    private static final int IMAGE_WIDTH = 640;
    private static final int IMAGE_HEIGHT = 480;
    private static final double CAMERA_FOV_HORIZONTAL = 60.0;   // Degrees
    private static final double BALL_DIAMETER_INCHES = 3.0;     // Actual ball size
    private static final double CAMERA_HEIGHT_INCHES = 9.0;     // Camera height above ground
    
    // Movement parameters
    private static final double APPROACH_SPEED = 0.4;
    private static final double SEARCH_SPEED = 0.25;
    private static final double FINE_APPROACH_DISTANCE = 12.0;  // Switch to fine approach
    
    @Override
    public void runOpMode() {
        // Initialize systems
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        initializeCamera();
        
        // Display status
        telemetry.addData("Status", "Enhanced Ball Detection Ready");
        telemetry.addData("Target", "Purple or Green Balls");
        telemetry.addData("Approach Distance", "%.1f inches", TARGET_DISTANCE_INCHES);
        telemetry.addLine("Camera initializing...");
        telemetry.update();
        
        // Wait for camera to be ready
        sleep(2000);
        
        telemetry.addData("Camera", "Ready");
        telemetry.addData("Controls", "Press START to begin");
        telemetry.update();
        
        waitForStart();
        runtime.reset();
        
        if (opModeIsActive()) {
            executeEnhancedMission();
        }
        
        // Cleanup
        camera.stopStreaming();
    }
    
    /**
     * Initialize camera with OpenCV pipeline
     */
    private void initializeCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        
        camera = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        
        pipeline = new BallTrackingPipeline();
        camera.setPipeline(pipeline);
        
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(IMAGE_WIDTH, IMAGE_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
            
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", "Code: " + errorCode);
                telemetry.update();
            }
        });
    }
    
    /**
     * Execute the enhanced mission with precise ball tracking
     */
    private void executeEnhancedMission() {
        BallTarget currentTarget = null;
        double lastDetectionTime = 0;
        boolean missionComplete = false;
        
        while (opModeIsActive() && !missionComplete && runtime.seconds() < MAX_MISSION_TIME) {
            
            // Get the nearest ball from vision pipeline
            BallTarget nearestBall = pipeline.getNearestBall();
            
            if (nearestBall != null) {
                currentTarget = nearestBall;
                lastDetectionTime = runtime.seconds();
                
                telemetry.addData("Ball Found", nearestBall.color);
                telemetry.addData("Distance", "%.1f inches", nearestBall.distance);
                telemetry.addData("Angle", "%.1f degrees", nearestBall.angleDegrees);
                telemetry.addData("Ball Size", "%.0f pixels", nearestBall.pixelArea);
                
                // Check if mission is complete
                if (nearestBall.distance <= TARGET_DISTANCE_INCHES) {
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    telemetry.addData("Mission", "TARGET REACHED!");
                    missionComplete = true;
                    
                } else {
                    // Drive towards the ball with appropriate strategy
                    if (nearestBall.distance > FINE_APPROACH_DISTANCE) {
                        // Long range approach - use trajectories
                        approachBallWithTrajectory(nearestBall);
                    } else {
                        // Close range approach - use direct drive
                        approachBallDirect(nearestBall);
                    }
                }
                
            } else {
                // No ball detected
                double timeSinceLastDetection = runtime.seconds() - lastDetectionTime;
                
                if (currentTarget != null && timeSinceLastDetection < SEARCH_TIMEOUT) {
                    // Continue towards last known position
                    telemetry.addData("Status", "Lost ball - continuing to last position");
                    
                } else {
                    // Search for new ball
                    telemetry.addData("Status", "Searching for ball...");
                    telemetry.addData("Search Time", "%.1f seconds", timeSinceLastDetection);
                    searchForBall();
                    currentTarget = null;
                }
            }
            
            // Update pose information
            Pose2d pose = drive.localizer.getPose();
            telemetry.addData("Robot X", "%.1f", pose.position.x);
            telemetry.addData("Robot Y", "%.1f", pose.position.y);
            telemetry.addData("Robot Heading", "%.1f°", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("Mission Time", "%.1f / %.1f seconds", 
                runtime.seconds(), MAX_MISSION_TIME);
            
            telemetry.update();
            sleep(50);
        }
        
        // Final status
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        
        if (missionComplete) {
            telemetry.addData("Mission Result", "SUCCESS!");
        } else {
            telemetry.addData("Mission Result", "TIMEOUT");
        }
        telemetry.addData("Total Time", "%.1f seconds", runtime.seconds());
        telemetry.update();
        
        // Hold final position
        while (opModeIsActive()) {
            sleep(100);
        }
    }
    
    /**
     * Approach ball using Road Runner trajectory (for long distances)
     */
    private void approachBallWithTrajectory(BallTarget ball) {
        double moveDistance = Math.max(ball.distance - TARGET_DISTANCE_INCHES, 4.0);
        double angleRadians = Math.toRadians(ball.angleDegrees);
        
        // Calculate target position
        Vector2d robotPosition = drive.localizer.getPose().position;
        double robotHeading = drive.localizer.getPose().heading.toDouble();
        
        // Transform ball position to field coordinates
        Vector2d ballOffset = new Vector2d(
            moveDistance * Math.cos(angleRadians),
            moveDistance * Math.sin(angleRadians)
        );
        
        Vector2d fieldTarget = robotPosition.plus(new Vector2d(
            ballOffset.x * Math.cos(robotHeading) - ballOffset.y * Math.sin(robotHeading),
            ballOffset.x * Math.sin(robotHeading) + ballOffset.y * Math.cos(robotHeading)
        ));
        
        // Execute trajectory
        TrajectoryActionBuilder builder = drive.actionBuilder(drive.localizer.getPose())
            .strafeToLinearHeading(fieldTarget, robotHeading);
        
        Actions.runBlocking(builder.build());
    }
    
    /**
     * Approach ball using direct drive commands (for fine positioning)
     */
    private void approachBallDirect(BallTarget ball) {
        double forwardPower = APPROACH_SPEED * 0.7;
        double strafePower = 0;
        double rotatePower = ball.angleDegrees * 0.02; // Proportional steering
        
        // Limit rotation power
        rotatePower = Math.max(-0.3, Math.min(0.3, rotatePower));
        
        drive.setDrivePowers(new PoseVelocity2d(
            new Vector2d(forwardPower, strafePower), rotatePower));
    }
    
    /**
     * Search for ball by rotating
     */
    private void searchForBall() {
        drive.setDrivePowers(new PoseVelocity2d(
            new Vector2d(0, 0), SEARCH_SPEED));
    }
    
    /**
     * OpenCV pipeline for ball detection and tracking
     */
    static class BallTrackingPipeline extends OpenCvPipeline {
        private Mat hsvMat = new Mat();
        private Mat purpleMask = new Mat();
        private Mat greenMask = new Mat();
        private Mat combinedMask = new Mat();
        private Mat hierarchy = new Mat();
        
        private BallTarget nearestBall = null;
        
        // Color ranges in HSV
        private Scalar purpleLower = new Scalar(110, 50, 50);
        private Scalar purpleUpper = new Scalar(150, 255, 255);
        private Scalar greenLower = new Scalar(35, 50, 50);
        private Scalar greenUpper = new Scalar(85, 255, 255);
        
        @Override
        public Mat processFrame(Mat input) {
            // Convert to HSV color space
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
            
            // Create color masks
            Core.inRange(hsvMat, purpleLower, purpleUpper, purpleMask);
            Core.inRange(hsvMat, greenLower, greenUpper, greenMask);
            Core.bitwise_or(purpleMask, greenMask, combinedMask);
            
            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(combinedMask, contours, hierarchy, 
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            
            nearestBall = null;
            double maxArea = 0;
            
            // Process each contour
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                
                if (area > 200) { // Minimum area threshold
                    Rect boundingRect = Imgproc.boundingRect(contour);
                    
                    // Calculate centroid
                    Moments moments = Imgproc.moments(contour);
                    if (moments.m00 > 0) {
                        Point centroid = new Point(
                            moments.m10 / moments.m00,
                            moments.m01 / moments.m00
                        );
                        
                        // Select the largest (nearest) ball
                        if (area > maxArea) {
                            maxArea = area;
                            
                            // Determine color
                            String color = determineColor(centroid);
                            
                            // Calculate distance and angle
                            double distance = estimateDistance(boundingRect.height);
                            double angle = calculateAngle(centroid.x);
                            
                            nearestBall = new BallTarget(color, distance, angle, 
                                (int)centroid.x, (int)centroid.y, area);
                            
                            // Draw visualization
                            Scalar drawColor = color.equals("PURPLE") ? 
                                new Scalar(255, 0, 255) : new Scalar(0, 255, 0);
                            
                            Imgproc.rectangle(input, boundingRect.tl(), boundingRect.br(), 
                                drawColor, 3);
                            Imgproc.circle(input, centroid, 8, new Scalar(255, 255, 0), -1);
                            
                            // Add text
                            Imgproc.putText(input, color + " " + String.format("%.1f\"", distance),
                                new Point(boundingRect.x, boundingRect.y - 10),
                                Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, drawColor, 2);
                        }
                    }
                }
            }
            
            return input;
        }
        
        private String determineColor(Point centroid) {
            // Sample the HSV value at the centroid to determine color
            double[] hsvValue = hsvMat.get((int)centroid.y, (int)centroid.x);
            if (hsvValue != null && hsvValue.length >= 3) {
                double hue = hsvValue[0];
                if (hue >= 110 && hue <= 150) {
                    return "PURPLE";
                } else if (hue >= 35 && hue <= 85) {
                    return "GREEN";
                }
            }
            return "UNKNOWN";
        }
        
        private double estimateDistance(double pixelHeight) {
            // Distance estimation using pinhole camera model
            // distance = (real_object_height * focal_length) / pixel_height
            double focalLength = (IMAGE_HEIGHT / 2.0) / 
                Math.tan(Math.toRadians(CAMERA_FOV_HORIZONTAL / 2.0));
            
            return (BALL_DIAMETER_INCHES * focalLength) / pixelHeight;
        }
        
        private double calculateAngle(double centerX) {
            // Calculate horizontal angle from image center
            double pixelOffset = centerX - (IMAGE_WIDTH / 2.0);
            double degreesPerPixel = CAMERA_FOV_HORIZONTAL / IMAGE_WIDTH;
            return pixelOffset * degreesPerPixel;
        }
        
        public BallTarget getNearestBall() {
            return nearestBall;
        }
    }
    
    /**
     * Data class for ball tracking information
     */
    static class BallTarget {
        public final String color;
        public final double distance;
        public final double angleDegrees;
        public final int centerX;
        public final int centerY;
        public final double pixelArea;
        
        public BallTarget(String color, double distance, double angleDegrees, 
                         int centerX, int centerY, double pixelArea) {
            this.color = color;
            this.distance = distance;
            this.angleDegrees = angleDegrees;
            this.centerX = centerX;
            this.centerY = centerY;
            this.pixelArea = pixelArea;
        }
    }
}