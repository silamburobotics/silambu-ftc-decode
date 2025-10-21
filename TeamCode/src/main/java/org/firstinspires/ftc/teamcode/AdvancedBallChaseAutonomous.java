package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
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
 * Advanced ball detection OpMode using OpenCV for precise ball tracking
 * This version provides better accuracy for ball position and distance estimation
 */

@Autonomous(name = "Advanced Ball Chase", group = "Vision")
public class AdvancedBallChaseAutonomous extends LinearOpMode {
    
    private OpenCvCamera camera;
    private BallDetectionPipeline pipeline;
    private MecanumDrive drive;
    
    // Detection parameters
    private static final double BALL_APPROACH_DISTANCE = 10.0; // inches
    private static final double FIELD_OF_VIEW_DEGREES = 60.0; // camera horizontal FOV
    private static final double CAMERA_HEIGHT_INCHES = 8.0;
    private static final double BALL_DIAMETER_INCHES = 3.0;
    private static final int IMAGE_WIDTH = 640;
    private static final int IMAGE_HEIGHT = 480;
    
    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        
        initializeCamera();
        
        telemetry.addData("Status", "Camera initialized");
        telemetry.addData("Detection", "Looking for purple/green balls");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                BallInfo nearestBall = pipeline.getNearestBall();
                
                if (nearestBall != null) {
                    telemetry.addData("Ball Found", nearestBall.color);
                    telemetry.addData("Distance", "%.1f inches", nearestBall.distance);
                    telemetry.addData("Angle", "%.1f degrees", nearestBall.angle);
                    telemetry.addData("Center X", nearestBall.centerX);
                    telemetry.addData("Center Y", nearestBall.centerY);
                    
                    if (nearestBall.distance > BALL_APPROACH_DISTANCE) {
                        driveTowardsBall(nearestBall);
                    } else {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        telemetry.addData("Status", "Reached ball!");
                    }
                } else {
                    telemetry.addData("Status", "Searching...");
                    // Slow search rotation
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0.15));
                }
                
                telemetry.update();
                sleep(50);
            }
        }
        
        camera.stopStreaming();
    }
    
    private void initializeCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        
        camera = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        
        pipeline = new BallDetectionPipeline();
        camera.setPipeline(pipeline);
        
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(IMAGE_WIDTH, IMAGE_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
            
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });
    }
    
    private void driveTowardsBall(BallInfo ball) {
        // Calculate movement based on ball position
        double angleRadians = Math.toRadians(ball.angle);
        double moveDistance = Math.max(ball.distance - BALL_APPROACH_DISTANCE, 6.0);
        
        // Get current robot pose
        Pose2d currentPose = drive.localizer.getPose();
        
        // Calculate target position in robot coordinate system
        double targetX = moveDistance * Math.cos(angleRadians);
        double targetY = moveDistance * Math.sin(angleRadians);
        
        // Transform to field coordinates
        Vector2d robotPos = currentPose.position;
        double robotHeading = currentPose.heading.toDouble();
        
        Vector2d fieldTarget = robotPos.plus(new Vector2d(
            targetX * Math.cos(robotHeading) - targetY * Math.sin(robotHeading),
            targetX * Math.sin(robotHeading) + targetY * Math.cos(robotHeading)
        ));
        
        // Create trajectory
        TrajectoryActionBuilder builder = drive.actionBuilder(currentPose)
            .strafeToLinearHeading(fieldTarget, robotHeading);
        
        Actions.runBlocking(builder.build());
    }
    
    static class BallDetectionPipeline extends OpenCvPipeline {
        private Mat hsvMat = new Mat();
        private Mat purpleMask = new Mat();
        private Mat greenMask = new Mat();
        private Mat combinedMask = new Mat();
        private Mat hierarchy = new Mat();
        
        private BallInfo nearestBall = null;
        
        // HSV color ranges for purple and green balls
        private Scalar purpleLower = new Scalar(120, 50, 50);
        private Scalar purpleUpper = new Scalar(150, 255, 255);
        private Scalar greenLower = new Scalar(40, 50, 50);
        private Scalar greenUpper = new Scalar(80, 255, 255);
        
        @Override
        public Mat processFrame(Mat input) {
            // Convert to HSV for better color detection
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
            
            // Create masks for purple and green
            Core.inRange(hsvMat, purpleLower, purpleUpper, purpleMask);
            Core.inRange(hsvMat, greenLower, greenUpper, greenMask);
            
            // Combine masks
            Core.bitwise_or(purpleMask, greenMask, combinedMask);
            
            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            
            nearestBall = null;
            double maxArea = 0;
            
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                
                if (area > 100) { // Minimum area threshold
                    Rect boundingRect = Imgproc.boundingRect(contour);
                    
                    // Calculate circle properties
                    Moments moments = Imgproc.moments(contour);
                    Point center = new Point(moments.m10 / moments.m00, moments.m01 / moments.m00);
                    
                    // Estimate if this is the largest/nearest ball
                    if (area > maxArea) {
                        maxArea = area;
                        
                        // Determine color by checking which mask this contour belongs to
                        Mat purpleTest = new Mat();
                        Mat greenTest = new Mat();
                        Core.inRange(hsvMat, purpleLower, purpleUpper, purpleTest);
                        Core.inRange(hsvMat, greenLower, greenUpper, greenTest);
                        
                        String color = "UNKNOWN";
                        if (purpleTest.get((int)center.y, (int)center.x)[0] > 0) {
                            color = "PURPLE";
                        } else if (greenTest.get((int)center.y, (int)center.x)[0] > 0) {
                            color = "GREEN";
                        }
                        
                        // Calculate distance and angle
                        double distance = estimateDistance(area, boundingRect);
                        double angle = calculateAngle(center.x);
                        
                        nearestBall = new BallInfo(color, distance, angle, (int)center.x, (int)center.y, area);
                        
                        // Draw bounding box and center point
                        Imgproc.rectangle(input, boundingRect.tl(), boundingRect.br(), 
                                        color.equals("PURPLE") ? new Scalar(255, 0, 255) : new Scalar(0, 255, 0), 2);
                        Imgproc.circle(input, center, 5, new Scalar(255, 255, 0), -1);
                    }
                }
            }
            
            return input;
        }
        
        private double estimateDistance(double area, Rect boundingRect) {
            // Distance estimation based on bounding box size
            // Assuming ball has known diameter
            double pixelHeight = boundingRect.height;
            double realHeight = BALL_DIAMETER_INCHES;
            
            // Simple pinhole camera model
            // distance = (real_height * focal_length) / pixel_height
            // Using estimated focal length based on field of view
            double focalLength = (IMAGE_HEIGHT / 2.0) / Math.tan(Math.toRadians(FIELD_OF_VIEW_DEGREES / 2.0));
            
            return (realHeight * focalLength) / pixelHeight;
        }
        
        private double calculateAngle(double centerX) {
            // Calculate horizontal angle from camera center
            double pixelOffset = centerX - (IMAGE_WIDTH / 2.0);
            double pixelsPerDegree = IMAGE_WIDTH / FIELD_OF_VIEW_DEGREES;
            return pixelOffset / pixelsPerDegree;
        }
        
        public BallInfo getNearestBall() {
            return nearestBall;
        }
    }
    
    static class BallInfo {
        public final String color;
        public final double distance;
        public final double angle;
        public final int centerX;
        public final int centerY;
        public final double area;
        
        public BallInfo(String color, double distance, double angle, int centerX, int centerY, double area) {
            this.color = color;
            this.distance = distance;
            this.angle = angle;
            this.centerX = centerX;
            this.centerY = centerY;
            this.area = area;
        }
    }
}