package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import android.graphics.Canvas;
import java.util.ArrayList;

import java.util.List;
import java.util.concurrent.TimeUnit;
@Config
@TeleOp(name = "TeleOpDECODE", group = "TeleOp")
public class TeleOpDECODE extends LinearOpMode {
    
    // Declare motors
    private DcMotorEx indexor;
    private DcMotorEx intake;
    private DcMotorEx shooter;
    private DcMotorEx conveyor; // Fixed typo from "notor_converyor"
    
    // Declare mecanum drive motors
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    
    // Declare servos
    private CRServo shooterServo;
    private Servo triggerServo;
    
    // Declare speed indicator light (using servo for PWM control)
    private Servo speedLight;
    
    // Declare color sensors for indexor ball detection
    private NormalizedColorSensor colorSensorIntake;    // Position 1: Entry point
    private NormalizedColorSensor colorSensorFire;      // Position 2: Exit/firing point
    private NormalizedColorSensor colorSensorStore;     // Position 3: Middle section
    
    // AprilTag detection
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private BallDetectionProcessor ballDetector;
    
    // Variables to track button states
    private boolean previousX = false;
    private boolean previousA = false;
    private boolean previousY = false;
    private boolean previousB = false;
    private boolean isAlignedToTag = false; // Track if robot is aligned to AprilTag
    
    // Gamepad2 button states for operator controls
    private boolean previousX2 = false;
    private boolean previousY2 = false;
    private boolean previousB2 = false;
    private boolean previousA2 = false;
    
    // Shooter state tracking
    private boolean shooterIntentionallyRunning = false; // Track if shooter was intentionally started
    
    // Trigger servo management
    private boolean manualTriggerControl = false; // Track if trigger is under manual control
    private ElapsedTime triggerManualTimer = new ElapsedTime(); // Timer for manual control timeout
    
    // AprilTag auto-alignment
    private boolean autoAlignmentActive = false; // Track if auto-alignment is active
    private ElapsedTime autoAlignmentTimer = new ElapsedTime(); // Timer for alignment timeout
    
    // Ball detection and alignment
    private boolean ballAlignmentActive = false; // Track if ball alignment is active
    private ElapsedTime ballAlignmentTimer = new ElapsedTime(); // Timer for ball alignment timeout
    private Point nearestBallCenter = null; // Detected ball center position
    private boolean ballDetected = false; // Track if any ball is detected
    
    // Auto-ball management system variables
    private boolean autoBallSystemEnabled = true;  // Enable automatic ball management
    private boolean previousBallAtIntake = false;  // Previous ball detection state
    private boolean previousBallAtFire = false;    // Previous ball detection at fire position
    private boolean previousBallAtStore = false;   // Previous ball detection at store position
    private ElapsedTime ballDetectionTimer = new ElapsedTime(); // Timer to debounce ball detection
    private boolean allPositionsFilled = false;    // Track if all three positions have balls
    private boolean intakeAutoStopped = false;     // Track if intake was auto-stopped
    
    // Indexor stuck detection variables
    private ElapsedTime indexorTimer = new ElapsedTime();
    private ElapsedTime indexorProgressTimer = new ElapsedTime();
    private int lastIndexorPosition = 0;
    private int indexorTargetPosition = 0;
    private boolean indexorIsRunning = false;
    private boolean indexorIsRecovering = false;
    private int indexorRecoveryAttempts = 0;
    private int indexorOriginalTarget = 0;
    
    // Motor power settings
    public static final double INTAKE_POWER = 0.8;
    public static final double SHOOTER_POWER = 1.0;
    public static final double CONVEYOR_POWER = 1.0;
    public static  int INDEXOR_TICKS = 179; // goBILDA 312 RPM motor: 120 degrees = 179 ticks
    
    // Indexor stuck detection settings
    public static final double INDEXOR_TIMEOUT = 1.0;       // seconds - max time for indexor to complete (reduced for faster recovery)
    public static final int INDEXOR_PROGRESS_THRESHOLD = 10; // minimum ticks of progress required
    public static final double INDEXOR_PROGRESS_CHECK_TIME = 1.0; // seconds between progress checks (reduced for faster detection)
    public static final int INDEXOR_REVERSE_TICKS = 90;     // 30 degrees reverse for stuck recovery (90 ticks ≈ 30°)
    
    // Servo power settings
    public static final double SHOOTER_SERVO_POWER = 1.0; // Positive for forward direction
    
    // Trigger servo position settings (0.0 = 0 degrees, 1.0 = 180 degrees)
    public static final double TRIGGER_FIRE = 0.25;     // 45 degrees (45/180 = 0.25) - FIRE POSITION (compact)
    public static final double TRIGGER_HOME = 0.76;     // 137 degrees (137/180 = 0.76) - HOME/SAFE POSITION (retracted)
    
    // AprilTag alignment settings
    public static final double HEADING_TOLERANCE = 2.0; // degrees
    
    // Speed light control settings (using servo positions for LED control)
    public static final double LIGHT_OFF_POSITION = 0.0;     // Servo position for light off
    public static final double LIGHT_GREEN_POSITION = 0.5;   // Servo position for green light
    public static final double LIGHT_WHITE_POSITION = 1.0;   // Servo position for white light (if needed)
    public static final double SHOOTER_SPEED_THRESHOLD = 0.95; // 95% of target speed
    
    // Mecanum drive settings
    public static final double DRIVE_SPEED_MULTIPLIER = 0.8;  // Max drive speed (0.0 to 1.0)
    public static final double STRAFE_SPEED_MULTIPLIER = 0.8; // Max strafe speed (0.0 to 1.0)
    public static final double TURN_SPEED_MULTIPLIER = 0.6;   // Max turn speed (0.0 to 1.0)
    
    // Color sensor settings for ball detection
    public static final double COLOR_SENSOR_GAIN = 2.0;      // Sensor gain for better detection
    public static final double BALL_DETECTION_THRESHOLD = 0.3; // Minimum alpha (proximity) for ball detection
    public static final double GREEN_BALL_THRESHOLD = 0.4;   // Green channel threshold for green ball detection
    public static final double PURPLE_BALL_THRESHOLD = 0.4;  // Combined red+blue threshold for purple ball detection
    
    // Auto-ball management system settings
    public static final double BALL_DETECTION_DEBOUNCE = 0.3; // Seconds to wait before confirming ball detection
    public static final double AUTO_INDEXOR_POWER = 0.3;      // Power for automatic indexor movement
    
    // Trigger servo automatic management settings
    public static final double MANUAL_TRIGGER_TIMEOUT = 3.0;  // Seconds before auto-management resumes
    
    // AprilTag auto-alignment settings
    public static final double AUTO_ALIGNMENT_TIMEOUT = 5.0;  // Seconds before auto-alignment stops
    public static final double ALIGNMENT_DRIVE_POWER = 0.3;   // Power for alignment movements
    public static final double ALIGNMENT_TURN_POWER = 0.25;   // Power for alignment rotation
    public static final double ALIGNMENT_POSITION_TOLERANCE = 2.0; // inches for position alignment
    public static final double ALIGNMENT_HEADING_TOLERANCE = 3.0;  // degrees for heading alignment
    
    // Camera-based alignment settings (pixel coordinates for 1280x720)
    public static final double CAMERA_CENTER_X = 640.0;      // Camera center X (1280/2)
    public static final double CAMERA_CENTER_Y = 360.0;      // Camera center Y (720/2)
    public static final double PIXEL_ALIGNMENT_TOLERANCE = 20.0; // pixels from center
    public static final double PIXEL_ALIGNMENT_POWER = 0.01;    // Power per pixel error (increased from 0.002)
    
    // Ball detection and alignment settings
    public static final double BALL_ALIGNMENT_TIMEOUT = 8.0;    // Seconds before ball alignment stops
    public static final double BALL_ALIGNMENT_DRIVE_POWER = 0.25; // Power for ball alignment movements
    public static final double BALL_ALIGNMENT_TURN_POWER = 0.2;   // Power for ball alignment rotation
    public static final double BALL_PIXEL_TOLERANCE = 30.0;      // pixels from center for ball alignment
    public static final double BALL_PIXEL_POWER = 0.008;         // Power per pixel error for ball alignment
    
    // Bearing-based alignment settings (angle-based alignment)
    public static final double BEARING_ALIGNMENT_TOLERANCE = 5.0; // degrees - target bearing tolerance
    public static final double BEARING_ALIGNMENT_POWER = 0.02;   // Power per degree of bearing error
    
    // AprilTag distance-based settings and thresholds
    public static final double OPTIMAL_SHOOTING_DISTANCE = 18.0;  // inches - optimal distance for shooting
    public static final double MIN_SHOOTING_DISTANCE = 8.0;       // inches - minimum safe shooting distance
    public static final double MAX_SHOOTING_DISTANCE = 36.0;      // inches - maximum effective shooting distance
    public static final double DISTANCE_TOLERANCE = 3.0;          // inches - tolerance for "at optimal distance"
    public static final double DISTANCE_APPROACH_POWER = 0.3;     // Power for distance-based approach movements
    
    // Shooter velocity control (ticks per second)
    public static double SHOOTER_TARGET_VELOCITY = 1600; // Range: 1200-1800 ticks/sec
    
    // Motor encoder specifications for RPM calculations
    // Common goBILDA motor encoder resolutions (ticks per revolution):
    // - 312 RPM motor: ~1425.2 ticks/rev (28 ticks/rev * 50.9:1 gear ratio)  
    // - 435 RPM motor: ~1020.0 ticks/rev (28 ticks/rev * 36.4:1 gear ratio)
    // - 1150 RPM motor: ~384.5 ticks/rev (28 ticks/rev * 13.7:1 gear ratio)
    // - 1620 RPM motor: ~312.0 ticks/rev (28 ticks/rev * 11.1:1 gear ratio)
    // Assuming 435 RPM motor for shooter (common for FTC shooters)
    public static final double SHOOTER_TICKS_PER_REVOLUTION = 1020.0; // goBILDA 435 RPM motor
    
    // AprilTag alignment settings
    public static final int TARGET_TAG_ID = 20; // Blue AprilTag ID
    public static final double ALIGNMENT_TOLERANCE = 5.0; // pixels
    public static final double ALIGNMENT_POWER = 0.3; // Power for alignment movements
    public static final double MIN_TAG_AREA = 100.0; // Minimum tag area for reliable detection
    
    @Override
    public void runOpMode() {
        // Initialize motors
        initializeMotors();
        
        // Initialize AprilTag detection
        initializeAprilTag();
        
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("=== GAMEPAD 1 (DRIVER) ===", "");
        telemetry.addData("A Button", "Intake + Converyor");
        telemetry.addData("X Button", "⚽ Align to Nearest Ball (Arducam)");
        telemetry.addData("Left Stick", "Drive/Strafe");
        telemetry.addData("Right Stick X", "Turn");
        telemetry.addData("Back Button", "Toggle Auto-Ball System");
        telemetry.addData("=== GAMEPAD 2 (OPERATOR) ===", "");
        telemetry.addData("A Button", "Auto-Align to AprilTag 20");
        telemetry.addData("X Button", "Indexor (120 degrees)");
        telemetry.addData("Y Button", "Shooter + Shooter Servo");
        telemetry.addData("B Button", "Trigger Servo (fire ↔ retract)");
        telemetry.addData("AprilTag", "Looking for Blue ID %d", TARGET_TAG_ID);
        telemetry.addData("🤖 Auto-Ball System", "Auto-advances balls when intake running");
        telemetry.addData("🤖 Auto-Trigger", "Keeps trigger HOME when intake running");
        telemetry.update();
        
        waitForStart();
        
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            handleControllerInputs();
            if (!autoAlignmentActive && !ballAlignmentActive) {
                handleMecanumDrive(); // Only manual drive if not auto-aligning
            }
            handleAprilTagAlignment();
            handleAutoAlignment(); // AprilTag auto-alignment system
            handleBallAlignment(); // Ball detection and alignment system
            showAprilTagDebugInfo(); // Always show detected tag IDs
            updateBallDetectionTelemetry(); // Show ball detection info
            checkIndexorCompletion();
            readColorSensors();
            handleAutoBallManagement(); // Automatic ball management system
            manageTriggerPosition(); // Automatic trigger position management
            updateSpeedLight();
            updateTelemetry();
            sleep(20); // Small delay to prevent excessive CPU usage
        }
        
        // Cleanup: Close vision portal to free resources
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
    
    private void initializeMotors() {
        // Initialize all motors
        indexor = hardwareMap.get(DcMotorEx.class, "indexor");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        conveyor = hardwareMap.get(DcMotorEx.class, "conveyor");
        
        // Initialize mecanum drive motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        
        // Initialize servos
        shooterServo = hardwareMap.get(CRServo.class, "shooterServo");
        triggerServo = hardwareMap.get(Servo.class, "triggerServo");
        
        // Initialize speed indicator light (using servo)
        speedLight = hardwareMap.get(Servo.class, "speedLight");
        
        // Set motor directions (adjust as needed for your robot)
        indexor.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        conveyor.setDirection(DcMotor.Direction.REVERSE);
        
        // Set mecanum drive motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        
        // Set servo directions
        shooterServo.setDirection(DcMotorSimple.Direction.REVERSE); // Back to original setting
        triggerServo.setDirection(Servo.Direction.FORWARD);
        speedLight.setDirection(Servo.Direction.FORWARD);
        
        // Set zero power behavior
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Set mecanum drive motor zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize servos to starting positions
        shooterServo.setPower(0);
        triggerServo.setPosition(TRIGGER_HOME); // Start at HOME position (safe position - 137 degrees)
        
        // Initialize speed light to off
        speedLight.setPosition(LIGHT_OFF_POSITION); // Start with light off
        
        // Reset encoders
        indexor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set mecanum drive motors to run without encoders for teleop
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set indexor to use encoder
        indexor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set shooter to use velocity control
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Initialize color sensors for indexor ball detection
        colorSensorIntake = hardwareMap.get(NormalizedColorSensor.class, "colorSensorEntry");
        colorSensorFire = hardwareMap.get(NormalizedColorSensor.class, "colorSensorExit");
        colorSensorStore = hardwareMap.get(NormalizedColorSensor.class, "colorSensorMiddle");
        
        // Set color sensor gain for better detection
        colorSensorIntake.setGain((float)COLOR_SENSOR_GAIN);
        colorSensorFire.setGain((float)COLOR_SENSOR_GAIN);
        colorSensorStore.setGain((float)COLOR_SENSOR_GAIN);
    }
    
    private void initializeAprilTag() {
        // Create the AprilTag processor
        // Initialize AprilTag processor WITHOUT tag library to detect any TAG_36h11 tags
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                // .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary()) // Comment out to detect any TAG_36h11
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                // Arducam OV9281 optimized lens intrinsics for 1280x720 resolution
                .setLensIntrinsics(1156.544, 1156.544, 640.0, 360.0) // fx, fy, cx, cy for 1280x720
                .build();

        // Create the vision portal with Arducam OV9281 specific settings
        VisionPortal.Builder builder = new VisionPortal.Builder();
        
        // Set the camera - Change this to match your hardware config name
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")); // Update this name if different
        
        // Arducam OV9281 optimal resolution settings
        // The OV9281 supports multiple resolutions, choose based on your needs:
        builder.setCameraResolution(new android.util.Size(1280, 720)); // Best for AprilTag detection (recommended)
        // builder.setCameraResolution(new Size(640, 480));  // Good balance of speed and accuracy
        // builder.setCameraResolution(new Size(320, 240));  // Maximum FPS for real-time tracking
        
        // Arducam OV9281 specific settings for optimal performance
        builder.enableLiveView(true); // Enable for debugging, disable for competition performance
        builder.setAutoStopLiveView(false); // Keep live view running
        
        // Initialize ball detection processor (for standalone use)
        ballDetector = new BallDetectionProcessor();
        
        // Set AprilTag processor
        builder.addProcessor(aprilTag);
        
        // Build the Vision Portal
        visionPortal = builder.build();
        
        // Configure Arducam OV9281 camera settings for optimal AprilTag detection
        configureArducamOV9281();
        
        telemetry.addData("Camera", "Arducam OV9281 Global Shutter");
        telemetry.addData("Resolution", "1280x720 (optimal for AprilTags)");
        telemetry.addData("AprilTag Vision", "Initialized with TAG_36h11 family");
        telemetry.addData("Tag Library", "NONE - Will detect ANY TAG_36h11 AprilTag");
        telemetry.addData("Target Tag", "Looking for ID %d", TARGET_TAG_ID);
        telemetry.addData("✅ Global Shutter", "No motion blur - perfect for moving robot");
        telemetry.update();
    }
    
    private void configureArducamOV9281() {
        // Wait for camera to be ready
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting for Arducam OV9281 to start streaming...");
            telemetry.update();
            sleep(50);
        }
        
        // Get camera control for Arducam OV9281 specific settings
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        FocusControl focusControl = visionPortal.getCameraControl(FocusControl.class);
        WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
        
        // Configure optimal settings for AprilTag detection with OV9281
        if (exposureControl != null) {
            // Set manual exposure for consistent detection
            exposureControl.setMode(ExposureControl.Mode.Manual);
            // OV9281 optimal exposure: 10-20ms for indoor lighting, 5-10ms for bright conditions
            exposureControl.setExposure(15, TimeUnit.MILLISECONDS); // Adjust based on your lighting
            telemetry.addData("Exposure", "Set to 15ms (manual mode)");
        }
        
        if (gainControl != null) {
            // Set moderate gain for good image quality without noise
            gainControl.setGain(50); // Range typically 0-255, adjust as needed
            telemetry.addData("Gain", "Set to 50");
        }
        
        if (focusControl != null) {
            // Set focus to infinity for AprilTag detection
            focusControl.setMode(FocusControl.Mode.Fixed);
            focusControl.setFocusLength(240.0); // Focus at distance for tags
            telemetry.addData("Focus", "Set to fixed 240.0");
        }
        
        if (whiteBalanceControl != null) {
            // Set white balance for consistent colors
            try {
                whiteBalanceControl.setWhiteBalanceTemperature(4000); // Indoor lighting (~4000K)
                telemetry.addData("White Balance", "Set to 4000K");
            } catch (Exception e) {
                telemetry.addData("White Balance", "Auto mode (manual not supported)");
            }
        }
        
        telemetry.addData("✅ Arducam OV9281", "Configured for optimal AprilTag detection");
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.update();
        
        // Brief pause to let settings take effect
        sleep(500);
    }
    
    private void handleAprilTagAlignment() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        
        // Reset alignment status
        isAlignedToTag = false;
        
        // Debug: Always show detection count
        telemetry.addData("👁️ Vision Status", "%d tags detected", currentDetections.size());
        
        // Look for the target tag
        for (AprilTagDetection detection : currentDetections) {
            if (detection != null && detection.id == TARGET_TAG_ID) {
                // Get distance and pose information
                double xOffset = detection.ftcPose.x;
                double yOffset = detection.ftcPose.y;
                double headingError = detection.ftcPose.yaw;
                double distance = detection.ftcPose.range; // Distance in inches
                double bearing = detection.ftcPose.bearing; // Bearing angle to tag
                double elevation = detection.ftcPose.elevation; // Elevation angle to tag
                
                // Check if we're aligned within tolerance
                boolean xAligned = Math.abs(xOffset) < ALIGNMENT_POSITION_TOLERANCE;
                boolean yAligned = Math.abs(yOffset) < ALIGNMENT_POSITION_TOLERANCE;
                boolean headingAligned = Math.abs(headingError) < ALIGNMENT_HEADING_TOLERANCE;
                
                isAlignedToTag = xAligned && yAligned && headingAligned;
                
                // Determine distance category for better feedback
                String distanceCategory;
                String distanceAdvice;
                if (distance < 100.0) {
                    distanceCategory = "� CLOSE RANGE";
                    distanceAdvice = "Within optimal range";
                } else {
                    distanceCategory = "� LONG RANGE";
                    distanceAdvice = "Beyond 100 inches";
                }
                
                // Adjust shooter velocity based on distance
                if (distance < 100.0) {
                    // Close range - use lower velocity
                    SHOOTER_TARGET_VELOCITY = 1200;
                } else {
                    // Long range - use higher velocity  
                    SHOOTER_TARGET_VELOCITY = 1600;
                }
                
                // Update shooter velocity if shooter is running
                if (shooterIntentionallyRunning && shooter != null) {
                    shooter.setVelocity(SHOOTER_TARGET_VELOCITY);
                }
                
                // Provide comprehensive feedback
                telemetry.addData("🎯 AprilTag", "FOUND Target ID %d", TARGET_TAG_ID);
                telemetry.addData("📍 Position", "X: %.1f, Y: %.1f inches", xOffset, yOffset);
                telemetry.addData("🧭 Heading Error", "%.1f degrees", headingError);
                telemetry.addData("📏 Distance", "%.1f inches (%s)", distance, distanceCategory);
                telemetry.addData("🚀 Shooter Speed", "%.0f ticks/sec (%.0f RPM)", 
                    SHOOTER_TARGET_VELOCITY, (SHOOTER_TARGET_VELOCITY * 60) / SHOOTER_TICKS_PER_REVOLUTION);
                telemetry.addData("🎯 Bearing", "%.1f degrees", bearing);
                telemetry.addData("📐 Elevation", "%.1f degrees", elevation);
                telemetry.addData("💡 Advice", distanceAdvice);
                telemetry.addData("✅ Alignment Status", 
                    "X: %s, Y: %s, Heading: %s", 
                    xAligned ? "✓" : "✗", 
                    yAligned ? "✓" : "✗", 
                    headingAligned ? "✓" : "✗");
                
                if (isAlignedToTag) {
                    telemetry.addData("🎯 READY TO FIRE", "Perfect alignment achieved!");
                } else {
                    telemetry.addData("⚙️ Status", "Manual adjustments needed...");
                }
                
                return; // Found our tag, no need to check others
            }
        }
        
        // If we get here, the target tag wasn't found
        telemetry.addData("🔍 AprilTag", "Searching for Target ID %d...", TARGET_TAG_ID);
        telemetry.addData("📋 Tags Detected", "%d total", currentDetections.size());
        
        // List any tags we can see for debugging
        for (AprilTagDetection detection : currentDetections) {
            if (detection != null) {
                telemetry.addData("👀 Visible Tag", "ID %d at %.1f inches (Meta: %s)", 
                    detection.id, detection.ftcPose.range, detection.metadata != null ? "Yes" : "No");
            }
        }
        
        if (currentDetections.size() == 0) {
            telemetry.addData("💡 Tip", "Make sure camera is working and tags are visible");
        }
    }
    
    private void handleControllerInputs() {
        // Get current button states for gamepad1 (driver)
        boolean currentA = gamepad1.a;
        boolean currentX = gamepad1.x; // Add X button for ball alignment
        
        // Get current button states for gamepad2 (operator)
        boolean currentX2 = gamepad2.x;
        boolean currentY2 = gamepad2.y;
        boolean currentB2 = gamepad2.b;
        boolean currentA2 = gamepad2.a;
        
        // Manual light testing with dpad (for debugging)
        if (gamepad1.dpad_up) {
            speedLight.setPosition(LIGHT_GREEN_POSITION);
            telemetry.addData("Manual Light Test", "GREEN position (%.2f)", LIGHT_GREEN_POSITION);
        } else if (gamepad1.dpad_down) {
            speedLight.setPosition(LIGHT_OFF_POSITION);
            telemetry.addData("Manual Light Test", "OFF position (%.2f)", LIGHT_OFF_POSITION);
        } else if (gamepad1.dpad_left) {
            speedLight.setPosition(LIGHT_WHITE_POSITION);
            telemetry.addData("Manual Light Test", "WHITE position (%.2f)", LIGHT_WHITE_POSITION);
        } else if (gamepad1.dpad_right) {
            speedLight.setPosition(0.75);
            telemetry.addData("Manual Light Test", "Test position (0.75)");
        }
        
        // Back button to toggle auto-ball management system
        if (gamepad1.back) {
            autoBallSystemEnabled = !autoBallSystemEnabled;
            ballDetectionTimer.reset();
            intakeAutoStopped = false;
            telemetry.addData("🤖 Auto-Ball System", autoBallSystemEnabled ? "ENABLED" : "DISABLED");
        }
        
        // Handle X button on gamepad2 - Run Indexor for 120 degrees
        if (currentX2 && !previousX2) {
            runIndexorToPosition(INDEXOR_TICKS);
        }
        
        // Handle A button on gamepad1 - Toggle Intake and Converyor
        if (currentA && !previousA) {
            toggleIntakeAndConveryor();
        }
        
        // Handle X button on gamepad1 - Toggle Ball Alignment using Arducam
        if (currentX && !previousX) {
            toggleBallAlignment();
        }
        
        // Handle A button on gamepad2 - Toggle AprilTag Auto-Alignment
        if (currentA2 && !previousA2) {
            toggleAutoAlignment();
        }
        
        // Handle Y button on gamepad2 - Toggle Shooter
        if (currentY2 && !previousY2) {
            toggleShooter();
        }
        
        // Handle B button on gamepad2 - Toggle Trigger Servo
        if (currentB2 && !previousB2) {
            toggleTriggerServo();
        }
        
        // Update previous button states
        previousA = currentA;
        previousX = currentX; // Add X button state tracking
        previousX2 = currentX2;
        previousY2 = currentY2;
        previousB2 = currentB2;
        previousA2 = currentA2;
    }
    
    private void handleMecanumDrive() {
        // Get joystick inputs
        double drive = -gamepad1.left_stick_y;  // Forward/backward (reversed: negative for forward - robot facing swapped)
        double strafe = -gamepad1.left_stick_x; // Left/right strafe (swapped: negative for right strafe)
        double turn = -gamepad1.right_stick_x;  // Rotation (swapped: negative for right turn)
        
        // Apply speed multipliers
        drive *= DRIVE_SPEED_MULTIPLIER;
        strafe *= STRAFE_SPEED_MULTIPLIER;
        turn *= TURN_SPEED_MULTIPLIER;
        
        // Calculate mecanum wheel powers
        double frontLeftPower = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower = drive - strafe + turn;
        double backRightPower = drive + strafe - turn;
        
        // Normalize powers to ensure they don't exceed 1.0
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                                  Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }
        
        // Apply powers to motors
        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftBack.setPower(backLeftPower);
        rightBack.setPower(backRightPower);
    }
    
    private void runIndexorToPosition(int ticks) {
        // Set target position
        int currentPosition = indexor.getCurrentPosition();
        int targetPosition = currentPosition + ticks;
        
        indexor.setTargetPosition(targetPosition);
        indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexor.setPower(0.3);
        
        // Start stuck detection tracking
        indexorTimer.reset();
        indexorProgressTimer.reset();
        lastIndexorPosition = currentPosition;
        indexorTargetPosition = targetPosition;
        indexorOriginalTarget = targetPosition; // Save original target for recovery
        indexorIsRunning = true;
        indexorIsRecovering = false;
        indexorRecoveryAttempts = 0;
        
        // Start conveyor to work with indexor
        conveyor.setPower(CONVEYOR_POWER);
        
        telemetry.addData("Indexor", "Moving to position: %d", targetPosition);
        telemetry.addData("Converyor", "RUNNING with indexor");
        telemetry.update();
    }
    
    private void checkIndexorCompletion() {
        // Only check if indexor is supposed to be running
        if (!indexorIsRunning || indexor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            return;
        }
        
        int currentPosition = indexor.getCurrentPosition();
        double timeElapsed = indexorTimer.seconds();
        double progressTime = indexorProgressTimer.seconds();
        
        // Check if indexor has completed successfully
        if (!indexor.isBusy()) {
            if (indexorIsRecovering) {
                // Recovery reverse movement completed, try original target again
                completeIndexorRecovery();
                return;
            } else {
                // Normal completion - indexor has reached target successfully
                indexorIsRunning = false;
                // Only stop conveyor if intake is not running (A button control)
                if (Math.abs(intake.getPower()) < 0.1) {
                    conveyor.setPower(0);
                    telemetry.addData("Indexor", "✓ COMPLETED successfully");
                    telemetry.addData("Converyor", "STOPPED - Indexor completed");
                }
                return;
            }
        }
        
        // STUCK DETECTION - Check for timeout
        if (timeElapsed > INDEXOR_TIMEOUT) {
            // Indexor has timed out - try recovery
            telemetry.addData("⚠️ INDEXOR TIMEOUT", "After %.1f seconds", timeElapsed);
            telemetry.addData("Position", "Current: %d, Target: %d", currentPosition, indexorTargetPosition);
            attemptIndexorRecovery();
            return;
        }
        
        // STUCK DETECTION - Check for lack of progress
        if (progressTime > INDEXOR_PROGRESS_CHECK_TIME) {
            int progressMade = Math.abs(currentPosition - lastIndexorPosition);
            
            if (progressMade < INDEXOR_PROGRESS_THRESHOLD) {
                // Not making sufficient progress - try recovery
                telemetry.addData("⚠️ INDEXOR NO PROGRESS", "In %.1f seconds", progressTime);
                telemetry.addData("Progress", "Only %d ticks (need %d)", progressMade, INDEXOR_PROGRESS_THRESHOLD);
                telemetry.addData("Position", "Current: %d, Target: %d", currentPosition, indexorTargetPosition);
                attemptIndexorRecovery();
                return;
            } else {
                // Making progress, reset progress timer and update position
                indexorProgressTimer.reset();
                lastIndexorPosition = currentPosition;
            }
        }
        
        // Normal operation - display progress
        int remaining = Math.abs(indexorTargetPosition - currentPosition);
        double percentComplete = (double)(Math.abs(currentPosition - (indexorTargetPosition - INDEXOR_TICKS))) / INDEXOR_TICKS * 100;
        
        if (indexorIsRecovering) {
            telemetry.addData("Indexor Status", "🔄 RECOVERY %d/2 - Backing out...", indexorRecoveryAttempts);
        } else {
            telemetry.addData("Indexor Status", "Moving... %.1f%% complete", Math.min(percentComplete, 100));
        }
        telemetry.addData("Position", "Current: %d, Target: %d (%d remaining)", 
                         currentPosition, indexorTargetPosition, remaining);
        telemetry.addData("Time", "%.1f / %.1f seconds", timeElapsed, INDEXOR_TIMEOUT);
        if (indexorRecoveryAttempts > 0) {
            telemetry.addData("Recovery Info", "Attempt %d/2 - Auto unstick system", indexorRecoveryAttempts);
        }
    }
    
    private void attemptIndexorRecovery() {
        if (indexorRecoveryAttempts >= 2) {
            // Too many recovery attempts, give up
            indexor.setPower(0);
            indexor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            indexorIsRunning = false;
            indexorIsRecovering = false;
            
            // Stop conveyor unless intake is running
            if (Math.abs(intake.getPower()) < 0.1) {
                conveyor.setPower(0);
            }
            
            telemetry.addData("❌ INDEXOR FAILED", "Recovery failed after %d attempts", indexorRecoveryAttempts);
            telemetry.addData("Action", "Manual intervention required - check for mechanical issues");
            return;
        }
        
        indexorRecoveryAttempts++;
        indexorIsRecovering = true;
        
        // Step 1: Back out 30 degrees (reverse)
        int currentPosition = indexor.getCurrentPosition();
        int reverseTarget = currentPosition - INDEXOR_REVERSE_TICKS;
        
        indexor.setTargetPosition(reverseTarget);
        indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexor.setPower(0.3);
        
        // Reset timers for recovery phase
        indexorTimer.reset();
        indexorProgressTimer.reset();
        lastIndexorPosition = currentPosition;
        indexorTargetPosition = reverseTarget;
        
        telemetry.addData("🔄 RECOVERY ATTEMPT", "%d/2 - Backing out 30°", indexorRecoveryAttempts);
        telemetry.addData("Recovery", "Moving from %d to %d", currentPosition, reverseTarget);
    }
    
    private void completeIndexorRecovery() {
        // Recovery reverse movement completed, now try original target again
        int currentPosition = indexor.getCurrentPosition();
        
        // Set target back to original goal
        indexor.setTargetPosition(indexorOriginalTarget);
        indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexor.setPower(0.3);
        
        // Reset tracking for retry
        indexorTimer.reset();
        indexorProgressTimer.reset();
        lastIndexorPosition = currentPosition;
        indexorTargetPosition = indexorOriginalTarget;
        indexorIsRecovering = false;
        
        telemetry.addData("🔄 RECOVERY", "Retry attempt %d - Moving to original target", indexorRecoveryAttempts);
        telemetry.addData("Target", "From %d to %d", currentPosition, indexorOriginalTarget);
    }
    
    private void toggleIntakeAndConveryor() {
        // Check if either motor is running
        boolean isRunning = (Math.abs(intake.getPower()) > 0.1) || 
                           (Math.abs(conveyor.getPower()) > 0.1);
        
        if (isRunning) {
            // Stop both motors
            intake.setPower(0);
            conveyor.setPower(0);
            intakeAutoStopped = false; // Reset auto-stop flag when manually stopped
            telemetry.addData("Intake & Converyor", "STOPPED");
        } else {
            // Safety check: Don't start intake if trigger is in fire position
            double currentTriggerPosition = triggerServo.getPosition();
            boolean triggerInFirePosition = Math.abs(currentTriggerPosition - TRIGGER_FIRE) < 0.05;
            
            if (triggerInFirePosition) {
                telemetry.addData("🚫 SAFETY BLOCK", "Cannot start intake - Trigger in FIRE position!");
                telemetry.addData("Action Required", "Move trigger to retracted position first (Press B)");
            } else {
                // Start both motors
                intake.setPower(INTAKE_POWER);
                conveyor.setPower(CONVEYOR_POWER);
                intakeAutoStopped = false; // Reset auto-stop flag when manually started
                // Reset ball management system when intake starts
                ballDetectionTimer.reset();
                allPositionsFilled = false;
                telemetry.addData("Intake & Converyor", "RUNNING");
                telemetry.addData("🤖 Auto-Ball System", "Monitoring for balls...");
            }
        }
        telemetry.update();
    }
    
    private void toggleShooter() {
        // Check if shooter is intentionally running (not just coasting)
        if (shooterIntentionallyRunning) {
            // Stop shooter and shooter servo
            shooter.setVelocity(0);
            shooterServo.setPower(0);
            shooterIntentionallyRunning = false;
            // Turn off speed light when shooter stops
            speedLight.setPosition(LIGHT_OFF_POSITION);
            telemetry.addData("Shooter", "STOPPED");
            telemetry.addData("Shooter Servo", "STOPPED");
            telemetry.addData("Speed Light", "OFF");
        } else {
            // Start shooter with target velocity and shooter servo
            shooter.setVelocity(SHOOTER_TARGET_VELOCITY);
            shooterServo.setPower(SHOOTER_SERVO_POWER);
            shooterIntentionallyRunning = true;
            telemetry.addData("Shooter", "RUNNING at %.0f ticks/sec", SHOOTER_TARGET_VELOCITY);
            telemetry.addData("Shooter Servo", "RUNNING at %.2f power", SHOOTER_SERVO_POWER);
            telemetry.addData("Speed Light", "Monitoring speed...");
        }
        telemetry.update();
    }
    
    private void toggleTriggerServo() {
        // Check if intake is running - safety first!
        boolean intakeRunning = Math.abs(intake.getPower()) > 0.1;
        if (intakeRunning) {
            telemetry.addData("🚫 TRIGGER BLOCKED", "Cannot move trigger while intake is running");
            telemetry.addData("💡 Safety Tip", "Stop intake (A button) before using trigger");
            telemetry.update();
            return;
        }
        
        // Mark as manual control and reset timer
        manualTriggerControl = true;
        triggerManualTimer.reset();
        
        // Check for AprilTag alignment for optimal firing
        if (!isAlignedToTag) {
            telemetry.addData("Trigger Servo", "MANUAL MODE - No AprilTag alignment");
            telemetry.addData("Warning", "Manual trigger - alignment recommended");
        } else {
            telemetry.addData("Trigger Servo", "ALIGNED MODE - AprilTag detected");
        }
        
        // Check current position and toggle between HOME (safe) and FIRE positions
        double currentPosition = triggerServo.getPosition();
        
        if (Math.abs(currentPosition - TRIGGER_FIRE) < 0.1) {
            // Currently at FIRE position, move to HOME/safe position
            triggerServo.setPosition(TRIGGER_HOME);
            telemetry.addData("Trigger Servo", "MOVING TO HOME (safe position - 137°)");
            if (isAlignedToTag) {
                telemetry.addData("AprilTag", "Aligned - trigger moved to safe position");
            }
        } else {
            // Currently at HOME position, move to FIRE position 
            triggerServo.setPosition(TRIGGER_FIRE);
            telemetry.addData("Trigger Servo", "MOVING TO FIRE POSITION (45°)");
            telemetry.addData("🎯 READY", "Trigger in firing position!");
        }
        telemetry.update();
    }
    
    private void updateSpeedLight() {
        // Get current shooter velocity and target
        double currentVelocity = Math.abs(shooter.getVelocity());
        double targetVelocity = SHOOTER_TARGET_VELOCITY;
        double speedPercentage = targetVelocity > 0 ? currentVelocity / targetVelocity : 0;
        
        // Check if shooter is running and at 95% speed
        if (currentVelocity > 50 && speedPercentage > SHOOTER_SPEED_THRESHOLD) {
            // Turn light green when greater than 95% speed
            speedLight.setPosition(LIGHT_GREEN_POSITION);
        } else if (currentVelocity > 50) {
            // Shooter is running but not at full speed - set to white/intermediate position
            speedLight.setPosition(LIGHT_WHITE_POSITION);
        } else {
            // Shooter is stopped - ensure light is off
            speedLight.setPosition(LIGHT_OFF_POSITION);
        }
    }
    
    private void readColorSensors() {
        // Read all three color sensors
        NormalizedRGBA intakeColors = colorSensorIntake.getNormalizedColors();
        NormalizedRGBA fireColors = colorSensorFire.getNormalizedColors();
        NormalizedRGBA storeColors = colorSensorStore.getNormalizedColors();
        
        // Detect balls at each position
        boolean ballAtIntake = detectBall(intakeColors);
        boolean ballAtFire = detectBall(fireColors);
        boolean ballAtStore = detectBall(storeColors);
        
        // Add color sensor telemetry
        telemetry.addData("Color Sensors", "Intake | Fire | Store");
        telemetry.addData("Ball Detection", "%s | %s | %s", 
            ballAtIntake ? "BALL" : "----",
            ballAtFire ? "BALL" : "----", 
            ballAtStore ? "BALL" : "----");
        
        // Auto-ball management status
        boolean intakeRunning = Math.abs(intake.getPower()) > 0.1;
        if (autoBallSystemEnabled && intakeRunning) {
            telemetry.addData("🤖 Auto-Ball System", "ACTIVE - Managing balls");
            telemetry.addData("All Positions", allPositionsFilled ? "✅ FILLED" : "⏳ Loading...");
            if (allPositionsFilled) {
                telemetry.addData("Status", "🛑 Intake & Indexor auto-stopped - All loaded!");
            } else {
                telemetry.addData("Status", "🔄 Auto-advancing balls as detected");
            }
        } else if (autoBallSystemEnabled && intakeAutoStopped) {
            telemetry.addData("🤖 Auto-Ball System", "💤 STANDBY - All positions filled");
            telemetry.addData("Status", "Press A to restart intake");
        } else if (autoBallSystemEnabled) {
            telemetry.addData("🤖 Auto-Ball System", "💤 STANDBY - Start intake to activate");
        } else {
            telemetry.addData("🤖 Auto-Ball System", "DISABLED");
        }
        
        // Show ball colors if detected
        if (ballAtIntake) {
            String ballColor = getBallColor(intakeColors);
            telemetry.addData("Intake Ball Color", ballColor);
        }
        if (ballAtFire) {
            String ballColor = getBallColor(fireColors);
            telemetry.addData("Fire Ball Color", ballColor);
        }
        if (ballAtStore) {
            String ballColor = getBallColor(storeColors);
            telemetry.addData("Store Ball Color", ballColor);
        }
        
        // Show raw color values for debugging
        telemetry.addData("Intake RGBA", "R:%.2f G:%.2f B:%.2f A:%.2f", 
            intakeColors.red, intakeColors.green, intakeColors.blue, intakeColors.alpha);
        telemetry.addData("Fire RGBA", "R:%.2f G:%.2f B:%.2f A:%.2f", 
            fireColors.red, fireColors.green, fireColors.blue, fireColors.alpha);
        telemetry.addData("Store RGBA", "R:%.2f G:%.2f B:%.2f A:%.2f", 
            storeColors.red, storeColors.green, storeColors.blue, storeColors.alpha);
    }
    
    private boolean detectBall(NormalizedRGBA colors) {
        // A ball is detected if the alpha (proximity) value is above threshold
        return colors.alpha > BALL_DETECTION_THRESHOLD;
    }
    
    private String getBallColor(NormalizedRGBA colors) {
        // Determine ball color based on RGB values
        if (colors.green > GREEN_BALL_THRESHOLD && colors.green > colors.red && colors.green > colors.blue) {
            return "GREEN";
        } else if ((colors.red + colors.blue) > PURPLE_BALL_THRESHOLD && colors.red > 0.2 && colors.blue > 0.2 && colors.green < colors.red) {
            // Purple is a combination of red and blue with low green
            return "PURPLE";
        } else {
            return "UNKNOWN";
        }
    }
    
    private void handleAutoBallManagement() {
        // Only run auto-ball management if system is enabled and intake is running
        if (!autoBallSystemEnabled || Math.abs(intake.getPower()) < 0.1 || indexorIsRunning) {
            return;
        }
        
        // Read current ball positions
        NormalizedRGBA intakeColors = colorSensorIntake.getNormalizedColors();
        NormalizedRGBA fireColors = colorSensorFire.getNormalizedColors();
        NormalizedRGBA storeColors = colorSensorStore.getNormalizedColors();
        
        boolean ballAtIntake = detectBall(intakeColors);
        boolean ballAtFire = detectBall(fireColors);
        boolean ballAtStore = detectBall(storeColors);
        
        // Update all positions filled status
        allPositionsFilled = ballAtIntake && ballAtFire && ballAtStore;
        
        // If all positions are filled, auto-stop intake and indexor
        if (allPositionsFilled && !intakeAutoStopped) {
            intake.setPower(0);
            conveyor.setPower(0);
            // Stop indexor if it's running
            if (indexorIsRunning) {
                indexor.setPower(0);
                indexor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                indexorIsRunning = false;
                indexorIsRecovering = false;
            }
            intakeAutoStopped = true;
            telemetry.addData("🤖 AUTO-STOP", "All indexor positions filled! Intake & Indexor stopped.");
            telemetry.addData("Status", "🎯 Ready to fire! Press A to restart intake.");
            return;
        }
        
        // Check for new ball detection at intake (edge detection)
        boolean newBallAtIntake = ballAtIntake && !previousBallAtIntake;
        
        // Trigger indexor if new ball detected at intake and there's space to advance
        if (newBallAtIntake && ballDetectionTimer.seconds() > BALL_DETECTION_DEBOUNCE) {
            if (!ballAtFire || !ballAtStore) {
                // There's space in the indexor, advance the ball
                runAutoIndexor("New ball detected at intake");
                ballDetectionTimer.reset();
            }
        }
        
        // Check if we need to advance balls that are already in the system
        else if (ballDetectionTimer.seconds() > BALL_DETECTION_DEBOUNCE * 2) {
            // Ball at intake but fire position empty - advance
            if (ballAtIntake && !ballAtFire) {
                runAutoIndexor("Intake has ball, fire position empty");
                ballDetectionTimer.reset();
            }
            // Ball at fire but store position empty - advance
            else if (ballAtFire && !ballAtStore) {
                runAutoIndexor("Fire has ball, store position empty");
                ballDetectionTimer.reset();
            }
        }
        
        // Update previous states for edge detection
        previousBallAtIntake = ballAtIntake;
        previousBallAtFire = ballAtFire;
        previousBallAtStore = ballAtStore;
    }
    
    private void runAutoIndexor(String reason) {
        // Run indexor automatically with the standard tick count
        int currentPosition = indexor.getCurrentPosition();
        int targetPosition = currentPosition + INDEXOR_TICKS;
        
        indexor.setTargetPosition(targetPosition);
        indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexor.setPower(AUTO_INDEXOR_POWER);
        
        // Start stuck detection tracking
        indexorTimer.reset();
        indexorProgressTimer.reset();
        lastIndexorPosition = currentPosition;
        indexorTargetPosition = targetPosition;
        indexorOriginalTarget = targetPosition;
        indexorIsRunning = true;
        indexorIsRecovering = false;
        indexorRecoveryAttempts = 0;
        
        // Keep conveyor running since intake is already running
        if (Math.abs(conveyor.getPower()) < 0.1) {
            conveyor.setPower(CONVEYOR_POWER);
        }
        
        telemetry.addData("🤖 AUTO-INDEXOR", "Triggered: %s", reason);
        telemetry.addData("Indexor", "Auto-moving to position: %d", targetPosition);
    }
    
    private void manageTriggerPosition() {
        // Check if manual control timeout has expired
        if (manualTriggerControl && triggerManualTimer.seconds() > MANUAL_TRIGGER_TIMEOUT) {
            manualTriggerControl = false;
        }
        
        // Check current trigger position
        double currentTriggerPosition = triggerServo.getPosition();
        boolean triggerInFirePosition = Math.abs(currentTriggerPosition - TRIGGER_FIRE) < 0.05;
        
        // Stop intake if trigger is in fire position
        if (triggerInFirePosition) {
            boolean intakeWasRunning = Math.abs(intake.getPower()) > 0.1;
            if (intakeWasRunning) {
                intake.setPower(0);
                // Also stop conveyor unless indexor is running
                if (!indexorIsRunning) {
                    conveyor.setPower(0);
                }
                telemetry.addData("🚫 AUTO-SAFETY", "Intake STOPPED - Trigger in FIRE position");
            }
        }
        
        // Only auto-manage trigger if not under manual control
        if (!manualTriggerControl) {
            boolean intakeRunning = Math.abs(intake.getPower()) > 0.1;
            
            if (intakeRunning) {
                // Keep trigger in HOME position when intake is running (safety first!)
                if (Math.abs(currentTriggerPosition - TRIGGER_HOME) > 0.05) {
                    triggerServo.setPosition(TRIGGER_HOME);
                    telemetry.addData("🤖 AUTO-TRIGGER", "Set to HOME position (intake running - safety)");
                }
            }
            // Note: We don't auto-move to fire position to avoid accidental firing
        }
    }
    
    private void toggleAutoAlignment() {
        if (autoAlignmentActive) {
            // Stop auto-alignment
            autoAlignmentActive = false;
            // Stop all drive motors
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            telemetry.addData("🎯 Auto-Alignment", "STOPPED by operator");
        } else {
            // Start auto-alignment
            autoAlignmentActive = true;
            autoAlignmentTimer.reset();
            telemetry.addData("🎯 Auto-Alignment", "STARTED - Looking for AprilTag %d", TARGET_TAG_ID);
            telemetry.addData("💡 Tip", "Point camera toward target tag");
        }
        telemetry.update();
    }
    
    private void handleAutoAlignment() {
        // Check if auto-alignment should timeout
        if (autoAlignmentActive && autoAlignmentTimer.seconds() > AUTO_ALIGNMENT_TIMEOUT) {
            autoAlignmentActive = false;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            telemetry.addData("🎯 Auto-Alignment", "TIMEOUT - Manual control resumed");
            return;
        }
        
        // Only run auto-alignment if active
        if (!autoAlignmentActive) {
            return;
        }
        
        List<AprilTagDetection> currentDetections = null;
        boolean targetFound = false;
        
        // Check for null processor
        if (aprilTag == null) {
            telemetry.addData("❌ Alignment Error", "AprilTag processor is null");
            autoAlignmentActive = false;
            return;
        }
        
        try {
            currentDetections = aprilTag.getDetections();
        } catch (Exception e) {
            telemetry.addData("❌ Alignment Error", "Detection exception: %s", e.getMessage());
            autoAlignmentActive = false;
            return;
        }
        
        if (currentDetections == null) {
            telemetry.addData("❌ Alignment Error", "Detection list is null");
            autoAlignmentActive = false;
            return;
        }
        
        // Debug: Show total detections
        telemetry.addData("🔍 Detections", "%d tags found", currentDetections.size());
        
        // List all detected tags for debugging
        for (AprilTagDetection detection : currentDetections) {
            if (detection != null) {
                telemetry.addData("📷 Detected Tag", "ID %d (Metadata: %s)", 
                    detection.id, detection.metadata != null ? "Yes" : "No");
            }
        }
        
        // Look for the target tag
        for (AprilTagDetection detection : currentDetections) {
            // Accept detections with or without metadata (since we're not using tag library)
            if (detection != null && detection.id == TARGET_TAG_ID) {
                targetFound = true;
                
                // Get bearing angle (detection angle) - this is what we want to minimize
                double bearingAngle = detection.ftcPose.bearing;  // Angle to tag in degrees
                double range = detection.ftcPose.range;           // Distance to tag
                double elevation = detection.ftcPose.elevation;   // Up/down angle
                
                telemetry.addData("🎯 Target Found", "Tag %d", TARGET_TAG_ID);
                telemetry.addData("🧭 Bearing Angle", "%.1f degrees", bearingAngle);
                telemetry.addData("🎯 Target Tolerance", "±%.1f degrees", BEARING_ALIGNMENT_TOLERANCE);
                telemetry.addData("� Range", "%.1f inches", range);
                telemetry.addData("� Elevation", "%.1f degrees", elevation);
                
                // Calculate drive powers for bearing-based alignment
                double drivePower = 0;
                double strafePower = 0;
                double turnPower = 0;
                
                // Turn to minimize bearing angle (primary alignment method)
                if (Math.abs(bearingAngle) > BEARING_ALIGNMENT_TOLERANCE) {
                    // Turn to reduce bearing angle to zero
                    turnPower = bearingAngle * BEARING_ALIGNMENT_POWER;
                    // Clamp to max power
                    turnPower = Math.max(-ALIGNMENT_TURN_POWER, 
                               Math.min(ALIGNMENT_TURN_POWER, turnPower));
                    telemetry.addData("🔧 Turn Power", "%.3f (Bearing: %.1f°)", turnPower, bearingAngle);
                }
                
                // Optional: Use strafe for fine adjustment if bearing is small but not zero
                if (Math.abs(bearingAngle) <= BEARING_ALIGNMENT_TOLERANCE * 2 && Math.abs(bearingAngle) > BEARING_ALIGNMENT_TOLERANCE) {
                    strafePower = bearingAngle * BEARING_ALIGNMENT_POWER * 0.5;
                    strafePower = Math.max(-ALIGNMENT_DRIVE_POWER * 0.5, 
                                 Math.min(ALIGNMENT_DRIVE_POWER * 0.5, strafePower));
                    telemetry.addData("🔧 Fine Strafe", "%.3f (Bearing: %.1f°)", strafePower, bearingAngle);
                }
                
                // Add minimum power if error is significant but calculated power is too small
                double minTurnPower = 0.12;  // Minimum power to overcome static friction for turning
                
                if (Math.abs(bearingAngle) > BEARING_ALIGNMENT_TOLERANCE && Math.abs(turnPower) < minTurnPower) {
                    turnPower = Math.signum(turnPower) * minTurnPower;
                    telemetry.addData("🔧 Min Turn Power", "%.3f applied", turnPower);
                }
                
                // Check if aligned based on bearing angle AND distance
                boolean bearingAligned = Math.abs(bearingAngle) < BEARING_ALIGNMENT_TOLERANCE;
                double distanceError = range - OPTIMAL_SHOOTING_DISTANCE;
                boolean distanceAligned = Math.abs(distanceError) < DISTANCE_TOLERANCE;
                boolean fullyAligned = bearingAligned && distanceAligned;
                
                telemetry.addData("📊 Full Status", "Bearing: %s (%.1f°), Distance: %s (%.1f\")", 
                    bearingAligned ? "✅" : "❌", Math.abs(bearingAngle),
                    distanceAligned ? "✅" : "❌", Math.abs(distanceError));
                
                if (fullyAligned) {
                    // Perfect alignment achieved - both bearing and distance!
                    autoAlignmentActive = false;
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                    telemetry.addData("🎯 Auto-Alignment", "✅ PERFECT ALIGNMENT!");
                    telemetry.addData("🎯 Status", "Bearing: %.1f° | Distance: %.1f inches", 
                        Math.abs(bearingAngle), range);
                    telemetry.addData("🚀 READY", "Optimal position for shooting!");
                } else {
                    // Apply alignment movements
                    double frontLeftPower = drivePower + strafePower + turnPower;
                    double frontRightPower = drivePower - strafePower - turnPower;
                    double backLeftPower = drivePower - strafePower + turnPower;
                    double backRightPower = drivePower + strafePower - turnPower;
                    
                    // Debug: Show if any power is being applied
                    telemetry.addData("🔍 Raw Powers", "D:%.3f S:%.3f T:%.3f", drivePower, strafePower, turnPower);
                    
                    leftFront.setPower(frontLeftPower);
                    rightFront.setPower(frontRightPower);
                    leftBack.setPower(backLeftPower);
                    rightBack.setPower(backRightPower);
                    
                    telemetry.addData("🎯 Auto-Alignment", "⚙️ ALIGNING TO BEARING...");
                    telemetry.addData("🔧 Motor Powers", "FL:%.3f FR:%.3f BL:%.3f BR:%.3f", 
                        frontLeftPower, frontRightPower, backLeftPower, backRightPower);
                    
                    // Alert if no significant power is being applied
                    double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                                             Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
                    if (maxPower < 0.05) {
                        telemetry.addData("⚠️ Warning", "Motor powers very low - check calculations");
                    }
                }
                
                break; // Found our tag, no need to check others
            }
        }
        
        if (!targetFound) {
            // Target tag not found, stop movement
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            telemetry.addData("🎯 Auto-Alignment", "⚠️ SEARCHING for AprilTag %d...", TARGET_TAG_ID);
            telemetry.addData("💡 Help", "Make sure tag is visible and well-lit");
        }
    }

    private void updateTelemetry() {
        // Display motor status
        telemetry.addData("Motor Status", "");
        telemetry.addData("Indexor Position", indexor.getCurrentPosition());
        telemetry.addData("Indexor Power", "%.2f", indexor.getPower());
        telemetry.addData("Intake Power", "%.2f", intake.getPower());
        telemetry.addData("Shooter Velocity", "%.0f / %.0f ticks/sec", 
                         shooter.getVelocity(), SHOOTER_TARGET_VELOCITY);
        
        // Calculate and display RPM
        double currentShooterRPM = (shooter.getVelocity() * 60) / SHOOTER_TICKS_PER_REVOLUTION;
        double targetShooterRPM = (SHOOTER_TARGET_VELOCITY * 60) / SHOOTER_TICKS_PER_REVOLUTION;
        telemetry.addData("Shooter RPM", "%.0f / %.0f RPM", currentShooterRPM, targetShooterRPM);
        telemetry.addData("Shooter Status", shooterIntentionallyRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Shooter Power", "%.2f", shooter.getPower());
        telemetry.addData("Converyor Power", "%.2f", conveyor.getPower());
        telemetry.addData("Shooter Servo Power", "%.2f", shooterServo.getPower());
        telemetry.addData("Trigger Servo Position", "%.3f (%.0f°)", 
                         triggerServo.getPosition(), triggerServo.getPosition() * 180);
        
        // Trigger management status
        boolean intakeRunning = Math.abs(intake.getPower()) > 0.1;
        if (manualTriggerControl) {
            double timeLeft = MANUAL_TRIGGER_TIMEOUT - triggerManualTimer.seconds();
            telemetry.addData("🎮 Trigger Control", "MANUAL (%.1fs left)", Math.max(0, timeLeft));
        } else if (intakeRunning) {
            telemetry.addData("🤖 Trigger Control", "AUTO - HOME position (intake running)");
        } else {
            telemetry.addData("🤖 Trigger Control", "AUTO - Ready for manual control");
        }
        
        // Display speed light status
        double currentVelocity = Math.abs(shooter.getVelocity());
        double speedPercentage = SHOOTER_TARGET_VELOCITY > 0 ? (currentVelocity / SHOOTER_TARGET_VELOCITY) * 100 : 0;
        telemetry.addData("Shooter Speed", "%.1f%% (%.0f/%.0f)", 
                         speedPercentage, currentVelocity, SHOOTER_TARGET_VELOCITY);
        
        // Enhanced speed light status with servo position
        double lightPosition = speedLight.getPosition();
        if (currentVelocity > 50 && speedPercentage > (SHOOTER_SPEED_THRESHOLD * 100)) {
            telemetry.addData("Speed Light", "GREEN - Ready! (pos: %.2f)", lightPosition);
        } else if (currentVelocity > 50) {
            telemetry.addData("Speed Light", "WHITE - Spinning up... (%.1f%%, pos: %.2f)", 
                             speedPercentage, lightPosition);
        } else {
            telemetry.addData("Speed Light", "OFF - Stopped (pos: %.2f)", lightPosition);
        }
        
        // Display drive motor status
        telemetry.addData("", "");
        telemetry.addData("Drive Motors", "");
        telemetry.addData("Left Front", "%.2f", leftFront.getPower());
        telemetry.addData("Right Front", "%.2f", rightFront.getPower());
        telemetry.addData("Left Back", "%.2f", leftBack.getPower());
        telemetry.addData("Right Back", "%.2f", rightBack.getPower());
        
        // Display button instructions
        telemetry.addData("", "");
        telemetry.addData("=== GAMEPAD 1 (DRIVER) ===", "");
        telemetry.addData("A Button", "Toggle Intake + Converyor");
        telemetry.addData("Left Stick", "Drive Forward/Back & Strafe Left/Right");
        telemetry.addData("Right Stick X", "Turn Left/Right");
        telemetry.addData("Back Button", "Toggle Auto-Ball Management System");
        telemetry.addData("DPad", "Manual Light Tests");
        telemetry.addData("", "");
        telemetry.addData("=== GAMEPAD 2 (OPERATOR) ===", "");
        telemetry.addData("A Button", "Toggle Auto-Align to AprilTag 20");
        telemetry.addData("X Button", "Move Indexor 120 degrees");
        telemetry.addData("Y Button", "Toggle Shooter + Shooter Servo");
        telemetry.addData("B Button", "Toggle Trigger Servo (60-120°)");
        telemetry.addData("DPad Up", "Manual Green Light Test");
        telemetry.addData("DPad Down", "Manual Light Off Test");
        telemetry.addData("DPad Left", "Manual White Light Test");
        telemetry.addData("DPad Right", "Manual Test Position");
        telemetry.addData("Back Button", "Toggle Auto-Ball Management System");
        
        // Enhanced indexor status with stuck detection info
        if (indexorIsRunning && indexor.isBusy()) {
            double timeElapsed = indexorTimer.seconds();
            telemetry.addData("Indexor Status", "⚙️ Moving... (%.1fs/%.1fs)", timeElapsed, INDEXOR_TIMEOUT);
            telemetry.addData("Stuck Detection", "Active - monitoring progress");
        } else if (indexorIsRunning && !indexor.isBusy()) {
            telemetry.addData("Indexor Status", "⚠️ Check stuck detection above");
        } else {
            telemetry.addData("Indexor Status", "Ready (Press X to move 120°)");
        }
        
        telemetry.update();
    }
    
    private void showAprilTagDebugInfo() {
        // Get current detections for debugging
        List<AprilTagDetection> currentDetections = null;
        
        // Check if aprilTag processor is null first
        if (aprilTag == null) {
            telemetry.addData("❌ AprilTag Error", "AprilTag processor is null!");
            telemetry.addData("💡 Fix", "Check initialization in runOpMode()");
            return;
        }
        
        try {
            currentDetections = aprilTag.getDetections();
        } catch (Exception e) {
            telemetry.addData("❌ Detection Error", "Exception: %s", e.getMessage());
            return;
        }
        
        // Always show detection summary at top level for easy debugging
        telemetry.addData("🔍 AprilTag Debug", "=== DETECTED TAGS ===");
        telemetry.addData("📊 Total Detections", "%d tags visible", currentDetections != null ? currentDetections.size() : 0);
        
        // Show vision portal status
        if (visionPortal != null) {
            telemetry.addData("📷 Camera Status", "Vision Portal: %s", visionPortal.getCameraState());
            telemetry.addData("🔧 Processing", "TAG_36h11 family, Target ID: %d", TARGET_TAG_ID);
        } else {
            telemetry.addData("❌ Camera Status", "Vision portal is null!");
        }
        
        if (currentDetections == null) {
            telemetry.addData("❌ Detections", "Detection list is null");
            return;
        }
        
        if (currentDetections.size() == 0) {
            telemetry.addData("❌ No Tags", "No AprilTags detected");
            telemetry.addData("💡 Check", "Camera angle, lighting, distance");
            telemetry.addData("🔍 Troubleshoot", "Make sure tag is TAG_36h11 family");
        } else {
            // Show detailed info for each detected tag
            for (int i = 0; i < currentDetections.size(); i++) {
                AprilTagDetection detection = currentDetections.get(i);
                
                // Check if detection has metadata
                if (detection == null) {
                    telemetry.addData(String.format("❌ Tag #%d", i + 1), "Detection is null");
                    continue;
                }
                
                if (detection.metadata == null) {
                    telemetry.addData(String.format("⚠️ Tag #%d", i + 1), 
                        "ID %d (No metadata - may not be in library)", detection.id);
                    continue;
                }
                
                boolean isTargetTag = (detection.id == TARGET_TAG_ID);
                String targetIndicator = isTargetTag ? " ⭐ TARGET" : "";
                
                telemetry.addData(String.format("🏷️ Tag #%d", i + 1), 
                    "ID %d%s", detection.id, targetIndicator);
                telemetry.addData(String.format("📏 Tag #%d Range", i + 1), 
                    "%.1f inches", detection.ftcPose.range);
                telemetry.addData(String.format("🧭 Tag #%d Bearing", i + 1), 
                    "%.1f degrees", detection.ftcPose.bearing);
                
                if (isTargetTag) {
                    telemetry.addData("🎯 Target Status", "FOUND! Ready for alignment");
                }
            }
            
            // Show list of all IDs in a compact format
            StringBuilder idList = new StringBuilder();
            for (AprilTagDetection detection : currentDetections) {
                if (detection != null) {
                    if (idList.length() > 0) idList.append(", ");
                    idList.append(detection.id);
                    if (detection.id == TARGET_TAG_ID) {
                        idList.append("⭐");
                    }
                    if (detection.metadata == null) {
                        idList.append("(no-meta)");
                    }
                }
            }
            telemetry.addData("📋 All Tag IDs", idList.toString());
        }
    }
    
    private void toggleBallAlignment() {
        try {
            if (ballAlignmentActive) {
                // Stop ball alignment
                ballAlignmentActive = false;
                // Stop all drive motors
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                telemetry.addData("⚽ Ball Alignment", "STOPPED by driver");
            } else {
                // Check if ball detector is available
                if (ballDetector == null) {
                    telemetry.addData("❌ Ball Alignment", "Ball detector not initialized");
                    telemetry.addData("💡 Solution", "Ball detection requires VisionProcessor integration");
                    telemetry.update();
                    return;
                }
                
                // Start ball alignment
                ballAlignmentActive = true;
                ballAlignmentTimer.reset();
                telemetry.addData("⚽ Ball Alignment", "STARTED - Looking for nearest ball");
                telemetry.addData("💡 Tip", "Point camera toward balls");
                telemetry.addData("⚠️ Note", "Ball detection is currently standalone - manual mode recommended");
            }
            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("❌ Ball Alignment Error", e.getMessage());
            telemetry.addData("🛠️ Debug", "Exception in toggleBallAlignment()");
            ballAlignmentActive = false;
            telemetry.update();
        }
    }
    
    private void handleBallAlignment() {
        try {
            // Check if ball alignment should timeout
            if (ballAlignmentActive && ballAlignmentTimer.seconds() > BALL_ALIGNMENT_TIMEOUT) {
                ballAlignmentActive = false;
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                telemetry.addData("⚽ Ball Alignment", "TIMEOUT - Manual control resumed");
                return;
            }
            
            // Only run ball alignment if active
            if (!ballAlignmentActive) {
                return;
            }
            
            // Check if ball detector is available
            if (ballDetector == null) {
                telemetry.addData("❌ Ball Detection", "Detector not available");
                ballAlignmentActive = false;
                return;
            }
            
            // Get ball detection results
            nearestBallCenter = ballDetector.getBallCenter();
            ballDetected = ballDetector.isBallDetected();
            
            if (ballDetected && nearestBallCenter != null) {
                // Calculate error from center
                double xError = nearestBallCenter.x - CAMERA_CENTER_X;
                double yError = nearestBallCenter.y - CAMERA_CENTER_Y;
                
                // Check if aligned
                boolean xAligned = Math.abs(xError) < BALL_PIXEL_TOLERANCE;
                boolean yAligned = Math.abs(yError) < BALL_PIXEL_TOLERANCE;
                
                if (xAligned && yAligned) {
                    // Perfectly aligned!
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                    telemetry.addData("⚽ Ball Alignment", "🎯 PERFECT ALIGNMENT!");
                    telemetry.addData("Status", "Ball centered - ready for intake");
                } else {
                    // Calculate alignment movements
                    double turnPower = xError * BALL_PIXEL_POWER; // Turn based on X error
                    double drivePower = yError * BALL_PIXEL_POWER; // Drive based on Y error
                    
                    // Limit powers
                    turnPower = Math.max(-BALL_ALIGNMENT_TURN_POWER, 
                               Math.min(BALL_ALIGNMENT_TURN_POWER, turnPower));
                    drivePower = Math.max(-BALL_ALIGNMENT_DRIVE_POWER, 
                                Math.min(BALL_ALIGNMENT_DRIVE_POWER, drivePower));
                    
                    // Apply mecanum drive for alignment
                    double frontLeftPower = drivePower + turnPower;
                    double frontRightPower = drivePower - turnPower;
                    double backLeftPower = drivePower + turnPower;
                    double backRightPower = drivePower - turnPower;
                    
                    leftFront.setPower(frontLeftPower);
                    rightFront.setPower(frontRightPower);
                    leftBack.setPower(backLeftPower);
                    rightBack.setPower(backRightPower);
                    
                    telemetry.addData("⚽ Ball Alignment", "🔄 ALIGNING TO BALL");
                    telemetry.addData("Ball Position", "X: %.0f, Y: %.0f", nearestBallCenter.x, nearestBallCenter.y);
                    telemetry.addData("Error", "X: %.0f, Y: %.0f pixels", xError, yError);
                    telemetry.addData("Powers", "Turn: %.2f, Drive: %.2f", turnPower, drivePower);
                }
            } else {
                // No ball detected
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                telemetry.addData("⚽ Ball Alignment", "🔍 SEARCHING FOR BALL...");
                telemetry.addData("Status", "No balls detected - adjust camera angle");
            }
        } catch (Exception e) {
            telemetry.addData("❌ Ball Alignment Error", e.getMessage());
            telemetry.addData("🛠️ Debug", "Exception in handleBallAlignment()");
            ballAlignmentActive = false;
            // Stop all motors as safety measure
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
        }
    }
    
    private double getOptimalShooterVelocity(double distance) {
        // Adjust shooter velocity based on distance to AprilTag
        // Base velocity for optimal distance (18 inches)
        double baseVelocity = SHOOTER_TARGET_VELOCITY;
        
        if (distance < MIN_SHOOTING_DISTANCE) {
            // Very close - reduce power to avoid overshooting
            return baseVelocity * 0.8;
        } else if (distance <= OPTIMAL_SHOOTING_DISTANCE + DISTANCE_TOLERANCE) {
            // In optimal range - use base velocity
            return baseVelocity;
        } else if (distance <= MAX_SHOOTING_DISTANCE) {
            // Far but still effective - increase power
            double powerMultiplier = 1.0 + ((distance - OPTIMAL_SHOOTING_DISTANCE) * 0.02);
            return Math.min(baseVelocity * powerMultiplier, baseVelocity * 1.3); // Cap at 130%
        } else {
            // Too far - maximum power but warn user
            return baseVelocity * 1.3;
        }
    }
    
    private String getDistanceAdvice(double distance) {
        if (distance < MIN_SHOOTING_DISTANCE) {
            return "⚠️ TOO CLOSE - Move back for safety";
        } else if (distance <= OPTIMAL_SHOOTING_DISTANCE - DISTANCE_TOLERANCE) {
            return "🔴 CLOSE - Good power range";
        } else if (distance <= OPTIMAL_SHOOTING_DISTANCE + DISTANCE_TOLERANCE) {
            return "🎯 PERFECT - Optimal shooting distance!";
        } else if (distance <= MAX_SHOOTING_DISTANCE) {
            return "🟡 FAR - Consider moving closer";
        } else {
            return "🔴 TOO FAR - Move much closer";
        }
    }
    
    private void updateBallDetectionTelemetry() {
        if (ballDetector != null) {
            boolean detected = ballDetector.isBallDetected();
            Point center = ballDetector.getBallCenter();
            
            telemetry.addData("👁️ Ball Detection", detected ? "BALL FOUND" : "No balls");
            if (detected && center != null) {
                telemetry.addData("📍 Ball Position", "X: %.0f, Y: %.0f", center.x, center.y);
                double distanceFromCenter = Math.sqrt(
                    Math.pow(center.x - CAMERA_CENTER_X, 2) + 
                    Math.pow(center.y - CAMERA_CENTER_Y, 2)
                );
                telemetry.addData("📏 Distance from Center", "%.0f pixels", distanceFromCenter);
            }
            
            if (ballAlignmentActive) {
                telemetry.addData("⚽ Ball Alignment", "ACTIVE (%.1fs)", ballAlignmentTimer.seconds());
                telemetry.addData("💡 Control", "Press X again to stop");
            } else {
                telemetry.addData("⚽ Ball Alignment", "Press X to align to nearest ball");
            }
        }
    }
    
    // Ball Detection Processor for Arducam OV9281
    private class BallDetectionProcessor {
        private Mat hsvMat = new Mat();
        private Mat maskMat = new Mat();
        private Mat hierarchyMat = new Mat();
        private Point detectedBallCenter = null;
        private boolean ballFound = false;
        
        // HSV color ranges for ball detection (adjust these for your balls)
        // Green ball HSV ranges
        private Scalar greenLowerBound = new Scalar(45, 50, 50);   // Lower HSV for green
        private Scalar greenUpperBound = new Scalar(75, 255, 255); // Upper HSV for green
        
        // Purple ball HSV ranges  
        private Scalar purpleLowerBound = new Scalar(120, 50, 50);   // Lower HSV for purple
        private Scalar purpleUpperBound = new Scalar(160, 255, 255); // Upper HSV for purple
        
        public Object processFrame(Mat frame, long captureTimeNanos) {
            // Convert BGR to HSV for better color detection
            Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
            
            // Create masks for green and purple balls
            Mat greenMask = new Mat();
            Mat purpleMask = new Mat();
            Mat combinedMask = new Mat();
            
            Core.inRange(hsvMat, greenLowerBound, greenUpperBound, greenMask);
            Core.inRange(hsvMat, purpleLowerBound, purpleUpperBound, purpleMask);
            Core.add(greenMask, purpleMask, combinedMask);
            
            // Find contours
            List<org.opencv.core.MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(combinedMask, contours, hierarchyMat, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            
            // Find the largest contour (closest/biggest ball)
            double maxArea = 0;
            Point ballCenter = null;
            
            for (org.opencv.core.MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea && area > 500) { // Minimum area threshold
                    maxArea = area;
                    
                    // Calculate centroid of the contour
                    Moments moments = Imgproc.moments(contour);
                    if (moments.m00 != 0) {
                        ballCenter = new Point(moments.m10 / moments.m00, moments.m01 / moments.m00);
                    }
                }
            }
            
            // Update detection results
            detectedBallCenter = ballCenter;
            ballFound = (ballCenter != null);
            
            // Draw detection on frame for debugging
            if (ballFound && ballCenter != null) {
                Imgproc.circle(frame, ballCenter, 10, new Scalar(0, 255, 0), 3);
                Imgproc.putText(frame, "BALL", 
                    new Point(ballCenter.x - 20, ballCenter.y - 20),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(0, 255, 0), 2);
            }
            
            // Draw center crosshairs
            Imgproc.line(frame, new Point(CAMERA_CENTER_X - 20, CAMERA_CENTER_Y), 
                        new Point(CAMERA_CENTER_X + 20, CAMERA_CENTER_Y), new Scalar(255, 0, 0), 2);
            Imgproc.line(frame, new Point(CAMERA_CENTER_X, CAMERA_CENTER_Y - 20), 
                        new Point(CAMERA_CENTER_X, CAMERA_CENTER_Y + 20), new Scalar(255, 0, 0), 2);
            
            // Clean up
            greenMask.release();
            purpleMask.release();
            combinedMask.release();
            
            return null;
        }
        
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            // Optional: Draw on canvas
        }
        
        // Public methods to get detection results
        public Point getBallCenter() {
            return detectedBallCenter;
        }
        
        public boolean isBallDetected() {
            return ballFound;
        }
    }
}