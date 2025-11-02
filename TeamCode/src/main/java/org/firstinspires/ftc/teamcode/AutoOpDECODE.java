package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import android.util.Size;
import java.util.List;

@Config
@Autonomous(name = "AutoOpDECODE", group = "Autonomous")
public class AutoOpDECODE extends LinearOpMode {
    
    // Declare motors - same as TeleOp
    private DcMotorEx indexor;
    private DcMotorEx intake;
    private DcMotorEx shooter;
    private DcMotorEx conveyor;
    
    // Declare mecanum drive motors
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    
    // Declare servos
    private CRServo shooterServo;
    private Servo triggerServo;
    private Servo speedLight;
    
    // AprilTag detection
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    
    // Ball detection sensor (using color sensor as proxy for ball detection)
    private ColorSensor ballSensor;
    private DistanceSensor distanceSensor;
    
    // Timing utilities
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stageTimer = new ElapsedTime();
    
    // Motor power settings - same as TeleOp
    public static final double INTAKE_POWER = 0.8;
    public static final double SHOOTER_POWER = 1.0;
    public static final double CONVEYOR_POWER = 1.0;
    public static final int INDEXOR_TICKS = 179; // goBILDA 312 RPM motor: 120 degrees = 179 ticks
    
    // Autonomous driving settings
    public static final double AUTO_DRIVE_SPEED = 0.4;     // Slow driving for precision
    public static final double AUTO_TURN_SPEED = 0.3;      // Slow turning for scanning
    public static final double BALL_SEARCH_SPEED = 0.2;    // Very slow for ball searching
    public static final double APPROACH_SPEED = 0.25;      // Speed when approaching targets
    
    // Servo settings - same as TeleOp
    public static final double SHOOTER_SERVO_POWER = -1.0; // Negative for reverse direction
    public static final double TRIGGER_FIRE = 0.333;    // 60 degrees (60/180 = 0.333)
    public static final double TRIGGER_HOME = 0.667;    // 120 degrees (120/180 = 0.667)
    public static final double LIGHT_OFF_POSITION = 0.0;
    public static final double LIGHT_GREEN_POSITION = 0.5;
    
    // Detection thresholds
    public static final double BALL_DETECTION_DISTANCE = 12.0;  // inches - distance to detect ball
    public static final double BALL_PICKUP_DISTANCE = 4.0;      // inches - distance to be over ball
    public static final int TARGET_TAG_ID = 20;                 // Blue AprilTag ID
    public static final double TAG_APPROACH_DISTANCE = 24.0;    // inches - stop 2 feet from tag
    public static final double TAG_ALIGNMENT_TOLERANCE = 3.0;   // inches - alignment tolerance
    
    // Field dimensions and positions (in inches)
    public static final double FIELD_WIDTH = 144.0;         // 12 feet = 144 inches
    public static final double FIELD_LENGTH = 144.0;        // 12 feet = 144 inches
    public static final double TILE_SIZE = 24.0;            // Each tile is 24x24 inches
    
    // Robot starting positions (choose one based on alliance and starting position)
    public static final double START_RED_LEFT_X = 12.0;     // Red alliance, left position
    public static final double START_RED_LEFT_Y = 12.0;
    public static final double START_RED_RIGHT_X = 132.0;   // Red alliance, right position
    public static final double START_RED_RIGHT_Y = 12.0;
    public static final double START_BLUE_LEFT_X = 12.0;    // Blue alliance, left position
    public static final double START_BLUE_LEFT_Y = 132.0;
    public static final double START_BLUE_RIGHT_X = 132.0;  // Blue alliance, right position
    public static final double START_BLUE_RIGHT_Y = 132.0;
    
    // Current blue robot starting position (change these values for your desired position)
    public static final double BLUE_ROBOT_START_X = START_BLUE_LEFT_X;   // Set to your X coordinate
    public static final double BLUE_ROBOT_START_Y = START_BLUE_LEFT_Y;   // Set to your Y coordinate
    
    // Key field positions
    public static final double CENTER_FIELD_X = 72.0;       // Center of field
    public static final double CENTER_FIELD_Y = 72.0;
    public static final double SCORING_ZONE_RED_Y = 24.0;   // Red scoring zone
    public static final double SCORING_ZONE_BLUE_Y = 120.0; // Blue scoring zone
    
    // Movement constants
    public static final double INCHES_PER_ENCODER_TICK = 0.05; // ⚠️ CALIBRATE THIS for your wheels!
    public static final double ROBOT_WIDTH = 18.0;          // Robot width in inches
    public static final double ROBOT_LENGTH = 18.0;         // Robot length in inches
    
    // Timing constants
    public static final double SCAN_TIMEOUT = 10.0;         // seconds - max time to scan for objects
    public static final double PICKUP_DURATION = 3.0;       // seconds - time to run intake for pickup
    public static final double SHOOTER_SPINUP_TIME = 2.0;   // seconds - time for shooter to reach speed
    public static final double FIRING_SEQUENCE_TIME = 1.5;  // seconds - time for firing sequence
    
    // COORDINATE TEST CONSTANTS - TEMPORARY FOR VERIFICATION
    public static final boolean ENABLE_COORDINATE_TEST = true; // Set to false to disable test mode
    public static final double TEST_DRIVE_SPEED = 0.3;         // Slower speed for better observation
    public static final double TEST_PAUSE_TIME = 5.0;          // Longer pause duration for better observation
    public static final double POSITION_TOLERANCE = 6.0;       // Slightly larger tolerance for testing
    
    // Test waypoints sequence - CENTER-BASED COORDINATE UNDERSTANDING TEST
    private static final double[][] TEST_WAYPOINTS = {
        {72.0, 72.0},   // Point 1 - CENTER (start here)
        {36.0, 108.0},  // Point 2 - TOP LEFT (from center: -36X, +36Y)
        {72.0, 72.0},   // Point 3 - CENTER (return)
        {108.0, 108.0}, // Point 4 - TOP RIGHT (from center: +36X, +36Y)
        {72.0, 72.0},   // Point 5 - CENTER (return)
        {36.0, 36.0},   // Point 6 - BOTTOM LEFT (from center: -36X, -36Y)
        {72.0, 72.0},   // Point 7 - CENTER (return)
        {108.0, 36.0},  // Point 8 - BOTTOM RIGHT (from center: +36X, -36Y)
        {72.0, 72.0}    // Point 9 - CENTER (final return)
    };
    
    // Ball collection constants for coordinate test
    public static final int TEST_TARGET_BALLS = 3;           // Number of balls to collect
    public static final double BALL_COLLECTION_TIMEOUT = 15.0; // Max time to collect balls
    public static final double SHOOTER_SPEED_THRESHOLD = 0.8;  // 80% of target speed
    
    // Shooter velocity control
    public static final double SHOOTER_TARGET_VELOCITY = 1600; // ticks/sec
    
    // Autonomous state tracking
    private enum AutoState {
        INIT,
        COORDINATE_TEST_COLLECT,  // NEW: Collect balls during coordinate test
        COORDINATE_TEST,          // NEW: Test coordinate system
        COORDINATE_TEST_SHOOT,    // NEW: Shooting phase after coordinate test
        SCAN_FOR_BALL,
        APPROACH_BALL,
        PICKUP_BALL,
        CHECK_PICKUP_COUNT,
        SCAN_FOR_TAG,
        APPROACH_TAG,
        PREPARE_SHOOTER,
        FIRE_SEQUENCE,
        RELOAD_SEQUENCE,
        CHECK_FIRE_COUNT,
        RETURN_TO_CENTER,
        COMPLETE
    }
    
    private AutoState currentState = AutoState.INIT;
    private boolean ballDetected = false;
    private boolean tagDetected = false;
    private AprilTagDetection targetTag = null;
    
    // Ball pickup tracking
    private int ballsCollected = 0;
    private final int MAX_BALLS_TO_COLLECT = 3;  // Repeat find and pick 3 times
    
    // Firing sequence tracking
    private int shotsFired = 0;
    private final int MAX_SHOTS_TO_FIRE = 3;     // Fire 3 times
    private double lastFireTime = 0;             // Track timing between shots
    
    // Field positioning tracking
    private double robotX = BLUE_ROBOT_START_X;  // Current robot X position
    private double robotY = BLUE_ROBOT_START_Y;  // Current robot Y position
    private double robotHeading = 0.0;          // Robot heading in degrees (0 = facing forward)
    
    // COORDINATE TEST TRACKING VARIABLES
    private int currentWaypointIndex = 0;        // Current test waypoint being navigated to
    private boolean waypointReached = false;     // Flag if current waypoint is reached
    private ElapsedTime pauseTimer = new ElapsedTime(); // Timer for pauses at waypoints
    private boolean isPausing = false;           // Flag if currently pausing at waypoint
    private int testBallsCollected = 0;          // Balls collected during coordinate test
    private ElapsedTime collectionTimer = new ElapsedTime(); // Timer for ball collection phase
    private boolean shooterReady = false;        // Flag if shooter is at target speed
    
    // Encoder tracking for position estimation
    private int lastLeftEncoder = 0;
    private int lastRightEncoder = 0;
    private int lastStrafeEncoder = 0;
    
    @Override
    public void runOpMode() {
        // Initialize all hardware
        initializeMotors();
        initializeAprilTag();
        initializeSensors();
        initializeFieldPosition();
        
        // Display initialization status
        telemetry.addData("Status", "AutoOpDECODE Initialized");
        telemetry.addData("Mission", "1. Find %d balls → 2. Pickup → 3. Find AprilTag → 4. Fire", MAX_BALLS_TO_COLLECT);
        telemetry.addData("Target", "Blue AprilTag ID %d", TARGET_TAG_ID);
        telemetry.addData("Starting Position", "X: %.1f, Y: %.1f", robotX, robotY);
        telemetry.addData("Field Size", "%.0f × %.0f inches", FIELD_WIDTH, FIELD_LENGTH);
        telemetry.addData("Ball Collection", "0 of %d collected", MAX_BALLS_TO_COLLECT);
        telemetry.addData("Ready", "Press PLAY to start autonomous");
        telemetry.update();
        
        waitForStart();
        runtime.reset();
        
        // Main autonomous loop
        while (opModeIsActive()) {
            updateRobotPosition();  // Track robot position using encoders
            updateDetections();
            executeCurrentState();
            updateTelemetry();
            sleep(50); // 20Hz update rate
        }
        
        // Cleanup
        stopAllMotors();
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
    
    private void initializeMotors() {
        // Initialize all motors - same as TeleOp
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
        speedLight = hardwareMap.get(Servo.class, "speedLight");
        
        // Set motor directions - same as TeleOp
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
        
        // Set zero power behavior to brake for precise autonomous movement
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize servos to starting positions
        shooterServo.setPower(0);
        triggerServo.setPosition(TRIGGER_FIRE);
        speedLight.setPosition(LIGHT_OFF_POSITION);
        
        // Reset encoders for precise movement
        resetDriveEncoders();
        indexor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Set run modes
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    private void initializeAprilTag() {
        // Create the AprilTag processor  
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Create the vision portal
        VisionPortal.Builder builder = new VisionPortal.Builder();
        
        // Set the camera
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        
        // Choose a camera resolution optimized for autonomous
        builder.setCameraResolution(new Size(640, 480));
        
        // Disable live view in autonomous to improve performance and avoid conflicts
        builder.enableLiveView(false);
        
        // Set and enable the processor
        builder.addProcessor(aprilTag);
        
        // Build the Vision Portal
        visionPortal = builder.build();
    }
    
    private void initializeSensors() {
        try {
            // Initialize sensors for ball detection
            ballSensor = hardwareMap.get(ColorSensor.class, "ballSensor");
            distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        } catch (Exception e) {
            telemetry.addData("Warning", "Ball sensors not found - using vision only");
        }
    }
    
    private void executeCurrentState() {
        switch (currentState) {
            case INIT:
                stageTimer.reset();
                ballsCollected = 0;  // Reset ball counter
                shotsFired = 0;      // Reset shot counter
                
                // Check if coordinate test mode is enabled
                if (ENABLE_COORDINATE_TEST) {
                    currentState = AutoState.COORDINATE_TEST_COLLECT;
                    currentWaypointIndex = 0;
                    waypointReached = false;
                    isPausing = false;
                    testBallsCollected = 0;
                    collectionTimer.reset();
                    telemetry.addData("🧪 Starting", "Ball Collection + Coordinate Test");
                } else {
                    currentState = AutoState.SCAN_FOR_BALL;
                }
                break;
                
            case COORDINATE_TEST_COLLECT:
                executeCoordinateTestCollect();
                break;
                
            case COORDINATE_TEST:
                executeCoordinateTest();
                break;
                
            case COORDINATE_TEST_SHOOT:
                executeCoordinateTestShoot();
                break;
                
            case SCAN_FOR_BALL:
                scanForBall();
                break;
                
            case APPROACH_BALL:
                approachBall();
                break;
                
            case PICKUP_BALL:
                pickupBall();
                break;
                
            case CHECK_PICKUP_COUNT:
                checkPickupCount();
                break;
                
            case SCAN_FOR_TAG:
                scanForAprilTag();
                break;
                
            case APPROACH_TAG:
                approachAprilTag();
                break;
                
            case PREPARE_SHOOTER:
                prepareShooter();
                break;
                
            case FIRE_SEQUENCE:
                executeFireSequence();
                break;
                
            case RELOAD_SEQUENCE:
                executeReloadSequence();
                break;
                
            case CHECK_FIRE_COUNT:
                checkFireCount();
                break;
                
            case RETURN_TO_CENTER:
                returnToCenter();
                break;
                
            case COMPLETE:
                stopAllMotors();
                break;
        }
    }
    
    private void scanForBall() {
        if (stageTimer.seconds() == 0) {
            telemetry.addData("Stage", "Scanning for ball %d of %d...", ballsCollected + 1, MAX_BALLS_TO_COLLECT);
            stageTimer.reset();
        }
        
        // Rotate robot slowly to scan for ball
        setMecanumPower(0, 0, AUTO_TURN_SPEED, 0);
        
        // Check for ball detection (using distance sensor or vision)
        if (isBallDetected()) {
            stopDriveMotors();
            ballDetected = true;
            currentState = AutoState.APPROACH_BALL;
            stageTimer.reset();
            telemetry.addData("Ball", "DETECTED! Approaching ball %d...", ballsCollected + 1);
        }
        
        // Timeout if no ball found - check if we should proceed anyway
        if (stageTimer.seconds() > SCAN_TIMEOUT) {
            stopDriveMotors();
            if (ballsCollected > 0) {
                // We have at least one ball, proceed to AprilTag
                telemetry.addData("Warning", "Ball %d scan timeout - proceeding with %d balls", 
                                ballsCollected + 1, ballsCollected);
                currentState = AutoState.SCAN_FOR_TAG;
            } else {
                // No balls collected yet, try AprilTag anyway
                telemetry.addData("Warning", "No balls found - proceeding to AprilTag");
                currentState = AutoState.SCAN_FOR_TAG;
            }
            stageTimer.reset();
        }
    }
    
    private void approachBall() {
        if (stageTimer.seconds() == 0) {
            telemetry.addData("Stage", "Approaching ball %d of %d...", ballsCollected + 1, MAX_BALLS_TO_COLLECT);
            stageTimer.reset();
        }
        
        // Drive toward ball with intake side facing forward
        double ballDistance = getBallDistance();
        
        if (ballDistance > BALL_PICKUP_DISTANCE) {
            // Move forward slowly toward ball
            setMecanumPower(APPROACH_SPEED, 0, 0, 0);
        } else {
            // Close enough to ball - stop and start pickup
            stopDriveMotors();
            currentState = AutoState.PICKUP_BALL;
            stageTimer.reset();
        }
        
        // Timeout safety - proceed to check count if we can't reach this ball
        if (stageTimer.seconds() > SCAN_TIMEOUT) {
            stopDriveMotors();
            currentState = AutoState.CHECK_PICKUP_COUNT;
            stageTimer.reset();
        }
    }
    
    private void pickupBall() {
        if (stageTimer.seconds() == 0) {
            telemetry.addData("Stage", "Picking up ball %d of %d...", ballsCollected + 1, MAX_BALLS_TO_COLLECT);
            // Start intake, conveyor, and indexor
            intake.setPower(INTAKE_POWER);
            conveyor.setPower(CONVEYOR_POWER);
            
            // Move indexor to pickup position
            int currentPosition = indexor.getCurrentPosition();
            indexor.setTargetPosition(currentPosition + INDEXOR_TICKS);
            indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            indexor.setPower(0.3);
            
            stageTimer.reset();
        }
        
        // Drive slowly over the ball to ensure pickup
        if (stageTimer.seconds() < 1.0) {
            setMecanumPower(BALL_SEARCH_SPEED, 0, 0, 0);
        } else {
            stopDriveMotors();
        }
        
        // Complete pickup sequence
        if (stageTimer.seconds() > PICKUP_DURATION) {
            // Stop intake systems
            intake.setPower(0);
            conveyor.setPower(0);
            
            // Increment ball counter
            ballsCollected++;
            telemetry.addData("Ball", "Pickup %d complete!", ballsCollected);
            currentState = AutoState.CHECK_PICKUP_COUNT;
            stageTimer.reset();
        }
    }
    
    private void checkPickupCount() {
        if (stageTimer.seconds() == 0) {
            telemetry.addData("Stage", "Checking pickup count...");
            stageTimer.reset();
        }
        
        // Check if we need to collect more balls
        if (ballsCollected < MAX_BALLS_TO_COLLECT) {
            telemetry.addData("Status", "Looking for ball %d of %d", ballsCollected + 1, MAX_BALLS_TO_COLLECT);
            ballDetected = false;  // Reset detection flag
            currentState = AutoState.SCAN_FOR_BALL;
            stageTimer.reset();
        } else {
            // Collected enough balls, proceed to AprilTag
            telemetry.addData("Collection", "All %d balls collected! Moving to AprilTag", MAX_BALLS_TO_COLLECT);
            currentState = AutoState.SCAN_FOR_TAG;
            stageTimer.reset();
        }
    }
    
    private void scanForAprilTag() {
        if (stageTimer.seconds() == 0) {
            telemetry.addData("Stage", "Scanning for AprilTag...");
            stageTimer.reset();
        }
        
        // Rotate slowly to find AprilTag
        setMecanumPower(0, 0, AUTO_TURN_SPEED, 0);
        
        // Check for AprilTag detection
        if (isTargetTagDetected()) {
            stopDriveMotors();
            tagDetected = true;
            currentState = AutoState.APPROACH_TAG;
            stageTimer.reset();
            telemetry.addData("AprilTag", "FOUND Blue ID %d!", TARGET_TAG_ID);
        }
        
        // Timeout if no tag found
        if (stageTimer.seconds() > SCAN_TIMEOUT) {
            stopDriveMotors();
            telemetry.addData("Warning", "AprilTag scan timeout - ending autonomous");
            currentState = AutoState.COMPLETE;
        }
    }
    
    private void approachAprilTag() {
        if (stageTimer.seconds() == 0) {
            telemetry.addData("Stage", "Approaching AprilTag...");
            stageTimer.reset();
        }
        
        if (targetTag != null) {
            double tagDistance = targetTag.ftcPose.range;
            double xOffset = targetTag.ftcPose.x;
            
            // Align and approach the tag
            if (tagDistance > TAG_APPROACH_DISTANCE) {
                // Move toward tag with alignment correction
                double forwardPower = APPROACH_SPEED;
                double strafePower = Math.max(-0.3, Math.min(0.3, xOffset * 0.05)); // Proportional steering
                
                setMecanumPower(forwardPower, strafePower, 0, 0);
            } else {
                // Close enough - stop and prepare to fire
                stopDriveMotors();
                telemetry.addData("Position", "2 feet from AprilTag - Ready to fire!");
                currentState = AutoState.PREPARE_SHOOTER;
                stageTimer.reset();
            }
        } else {
            // Lost the tag - go back to scanning
            currentState = AutoState.SCAN_FOR_TAG;
            stageTimer.reset();
        }
        
        // Timeout safety
        if (stageTimer.seconds() > SCAN_TIMEOUT) {
            stopDriveMotors();
            currentState = AutoState.PREPARE_SHOOTER;
            stageTimer.reset();
        }
    }
    
    private void prepareShooter() {
        if (stageTimer.seconds() == 0) {
            telemetry.addData("Stage", "Preparing shooter for %d shots...", MAX_SHOTS_TO_FIRE);
            // Start shooter and shooter servo
            shooter.setVelocity(SHOOTER_TARGET_VELOCITY);
            shooterServo.setPower(SHOOTER_SERVO_POWER);
            speedLight.setPosition(LIGHT_GREEN_POSITION);
            shotsFired = 0;  // Reset shot counter
            stageTimer.reset();
        }
        
        // Wait for shooter to reach target speed
        double currentVelocity = Math.abs(shooter.getVelocity());
        double speedPercentage = currentVelocity / SHOOTER_TARGET_VELOCITY;
        
        if (speedPercentage > 0.95 && stageTimer.seconds() > SHOOTER_SPINUP_TIME) {
            telemetry.addData("Shooter", "Ready! Starting fire sequence...");
            currentState = AutoState.FIRE_SEQUENCE;
            stageTimer.reset();
        }
        
        telemetry.addData("Shooter Speed", "%.1f%% (%.0f/%.0f)", 
                         speedPercentage * 100, currentVelocity, SHOOTER_TARGET_VELOCITY);
    }
    
    private void executeFireSequence() {
        if (stageTimer.seconds() == 0) {
            telemetry.addData("Stage", "FIRING shot %d of %d!", shotsFired + 1, MAX_SHOTS_TO_FIRE);
            lastFireTime = runtime.seconds();
            stageTimer.reset();
        }
        
        double sequenceTime = stageTimer.seconds();
        
        if (sequenceTime < 0.5) {
            // Start indexor and conveyor to feed ball
            conveyor.setPower(CONVEYOR_POWER);
            int currentPosition = indexor.getCurrentPosition();
            indexor.setTargetPosition(currentPosition + INDEXOR_TICKS);
            indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            indexor.setPower(0.3);
            
        } else if (sequenceTime < 1.0) {
            // Fire trigger servo
            triggerServo.setPosition(TRIGGER_HOME);
            telemetry.addData("Trigger", "SHOT %d FIRED!", shotsFired + 1);
            
        } else if (sequenceTime < 1.5) {
            // Reset trigger servo
            triggerServo.setPosition(TRIGGER_FIRE);
            
        } else {
            // Complete single shot, increment counter
            shotsFired++;
            telemetry.addData("Shot Complete", "%d of %d fired", shotsFired, MAX_SHOTS_TO_FIRE);
            
            // Check if we need more shots
            if (shotsFired < MAX_SHOTS_TO_FIRE) {
                currentState = AutoState.RELOAD_SEQUENCE;
                stageTimer.reset();
            } else {
                // All shots complete - return to center field
                currentState = AutoState.RETURN_TO_CENTER;
                stageTimer.reset();
            }
        }
    }
    
    private void executeReloadSequence() {
        if (stageTimer.seconds() == 0) {
            telemetry.addData("Stage", "Reloading for next shot...");
            stageTimer.reset();
        }
        
        double reloadTime = stageTimer.seconds();
        
        if (reloadTime < 0.8) {
            // Brief pause to let mechanisms settle
            conveyor.setPower(0);
            telemetry.addData("Reload", "Settling mechanisms...");
            
        } else if (reloadTime < 1.5) {
            // Run conveyor and indexor to position next ball
            conveyor.setPower(CONVEYOR_POWER * 0.7); // Slower for positioning
            int currentPosition = indexor.getCurrentPosition();
            indexor.setTargetPosition(currentPosition + (INDEXOR_TICKS / 2)); // Smaller movement
            indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            indexor.setPower(0.6);
            
            telemetry.addData("Reload", "Positioning next ball...");
            
        } else {
            // Reload complete, ready for next shot
            conveyor.setPower(0);
            telemetry.addData("Reload", "Ready for shot %d!", shotsFired + 1);
            currentState = AutoState.FIRE_SEQUENCE;
            stageTimer.reset();
        }
    }
    
    private void checkFireCount() {
        // This method is now handled within executeFireSequence
        // Keeping for potential future use or debugging
        if (shotsFired < MAX_SHOTS_TO_FIRE) {
            currentState = AutoState.RELOAD_SEQUENCE;
        } else {
            // All shots complete - return to center field
            stopAllMotors();
            speedLight.setPosition(LIGHT_OFF_POSITION);
            telemetry.addData("Mission", "ALL %d SHOTS FIRED! Returning to center...", MAX_SHOTS_TO_FIRE);
            currentState = AutoState.RETURN_TO_CENTER;
        }
        stageTimer.reset();
    }
    
    private void returnToCenter() {
        if (stageTimer.seconds() == 0) {
            telemetry.addData("Stage", "Returning to center field...");
            stageTimer.reset();
        }
        
        // Calculate distance to center field
        double deltaX = CENTER_FIELD_X - robotX;
        double deltaY = CENTER_FIELD_Y - robotY;
        double distanceToCenter = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        
        telemetry.addData("Return to Center", "Distance: %.1f inches", distanceToCenter);
        telemetry.addData("Current Position", "X: %.1f, Y: %.1f", robotX, robotY);
        telemetry.addData("Target Position", "X: %.1f, Y: %.1f", CENTER_FIELD_X, CENTER_FIELD_Y);
        
        // Check if we've reached the center (within tolerance)
        if (distanceToCenter < TILE_SIZE) {
            // Arrived at center - stop and complete mission
            stopAllMotors();
            telemetry.addData("Mission", "COMPLETE! Robot at center field.");
            telemetry.addData("Final Position", "X: %.1f, Y: %.1f", robotX, robotY);
            currentState = AutoState.COMPLETE;
            stageTimer.reset();
            return;
        }
        
        // Calculate movement direction
        double moveAngle = Math.atan2(deltaY, deltaX);
        double moveX = Math.cos(moveAngle) * APPROACH_SPEED;
        double moveY = Math.sin(moveAngle) * APPROACH_SPEED;
        
        // Use mecanum drive to move toward center
        setMecanumPower(moveY, moveX, 0, 0); // Forward, strafe, turn, (reserved)
        
        telemetry.addData("Movement", "Angle: %.1f°, Speed: %.2f", 
            Math.toDegrees(moveAngle), APPROACH_SPEED);
    }
    
    // Detection and sensor methods
    private boolean isBallDetected() {
        // Use distance sensor or color sensor to detect ball
        if (distanceSensor != null) {
            double distance = distanceSensor.getDistance(DistanceUnit.INCH);
            return distance < BALL_DETECTION_DISTANCE && distance > 2.0; // Valid range
        }
        
        // Fallback: use color sensor
        if (ballSensor != null) {
            // Detect based on color intensity or specific color
            int red = ballSensor.red();
            int green = ballSensor.green();
            int blue = ballSensor.blue();
            
            // Adjust these thresholds based on your ball color
            return (red + green + blue) > 500; // Basic object detection
        }
        
        return false; // No sensors available
    }
    
    private double getBallDistance() {
        if (distanceSensor != null) {
            return distanceSensor.getDistance(DistanceUnit.INCH);
        }
        return BALL_PICKUP_DISTANCE; // Default if no sensor
    }
    
    private boolean isTargetTagDetected() {
        updateDetections();
        return targetTag != null;
    }
    
    private void updateDetections() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        targetTag = null;
        
        // Look for the target tag
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == TARGET_TAG_ID) {
                targetTag = detection;
                break;
            }
        }
    }
    
    // Drive control methods
    private void setMecanumPower(double drive, double strafe, double turn, double heading) {
        // Calculate mecanum wheel powers
        double frontLeftPower = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower = drive - strafe + turn;
        double backRightPower = drive + strafe - turn;
        
        // Normalize powers
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
    
    private void stopDriveMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
    
    private void stopAllMotors() {
        stopDriveMotors();
        indexor.setPower(0);
        intake.setPower(0);
        shooter.setVelocity(0);
        conveyor.setPower(0);
        shooterServo.setPower(0);
    }
    
    private void resetDriveEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    private void setDriveMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        leftBack.setMode(mode);
        rightBack.setMode(mode);
    }
    
    private void updateTelemetry() {
        // Display current state and progress
        telemetry.addData("Autonomous State", currentState.toString());
        telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
        telemetry.addData("Stage Timer", "%.1f seconds", stageTimer.seconds());
        
        // COORDINATE TEST MODE DEBUGGING
        if (ENABLE_COORDINATE_TEST) {
            telemetry.addData("🧪 TEST MODE", "ACTIVE - Debug Info Below");
            telemetry.addData("Waypoint Progress", "%d / %d", currentWaypointIndex + 1, TEST_WAYPOINTS.length);
            if (currentWaypointIndex < TEST_WAYPOINTS.length) {
                double targetX = TEST_WAYPOINTS[currentWaypointIndex][0];
                double targetY = TEST_WAYPOINTS[currentWaypointIndex][1];
                double distance = Math.sqrt(Math.pow(targetX - robotX, 2) + Math.pow(targetY - robotY, 2));
                telemetry.addData("Current Target", "(%.1f, %.1f)", targetX, targetY);
                telemetry.addData("Distance to Target", "%.1f inches", distance);
                telemetry.addData("Reached?", waypointReached ? "YES" : "NO");
                telemetry.addData("Pausing?", isPausing ? "YES" : "NO");
            }
            telemetry.addData("🔧 Encoders", "L:%d R:%d S:%d", 
                            leftFront.getCurrentPosition(), 
                            rightFront.getCurrentPosition(), 
                            leftBack.getCurrentPosition());
        }
        
        // Display ball collection progress
        telemetry.addData("Balls Collected", "%d of %d", ballsCollected, MAX_BALLS_TO_COLLECT);
        if (ballsCollected < MAX_BALLS_TO_COLLECT) {
            telemetry.addData("Current Target", "Ball %d", ballsCollected + 1);
        } else {
            telemetry.addData("Collection Status", "All balls collected!");
        }
        
        // Display field position
        telemetry.addData("Robot Position", "X: %.1f, Y: %.1f", robotX, robotY);
        telemetry.addData("Robot Heading", "%.1f degrees", robotHeading);
        telemetry.addData("Field Zone", getFieldZone());
        
        // Display detection status
        telemetry.addData("Ball Detected", ballDetected ? "YES" : "NO");
        telemetry.addData("Tag Detected", tagDetected ? "YES" : "NO");
        
        if (targetTag != null) {
            telemetry.addData("AprilTag", "ID %d at %.1f inches", targetTag.id, targetTag.ftcPose.range);
            telemetry.addData("Tag Position", "X: %.1f, Y: %.1f", targetTag.ftcPose.x, targetTag.ftcPose.y);
        }
        
        // Display motor status
        telemetry.addData("Indexor Position", indexor.getCurrentPosition());
        telemetry.addData("Shooter Velocity", "%.0f / %.0f ticks/sec", 
                         shooter.getVelocity(), SHOOTER_TARGET_VELOCITY);
        
        // Display sensor readings if available
        if (distanceSensor != null) {
            telemetry.addData("Distance Sensor", "%.1f inches", 
                             distanceSensor.getDistance(DistanceUnit.INCH));
        }
        
        telemetry.update();
    }
    
    // Field positioning and navigation methods
    private void updateRobotPosition() {
        // Get current encoder values
        int currentLeftEncoder = leftFront.getCurrentPosition();
        int currentRightEncoder = rightFront.getCurrentPosition();
        int currentStrafeEncoder = leftBack.getCurrentPosition(); // Use left back for strafe estimation
        
        // Calculate encoder deltas
        int deltaLeft = currentLeftEncoder - lastLeftEncoder;
        int deltaRight = currentRightEncoder - lastRightEncoder;
        int deltaStrafe = currentStrafeEncoder - lastStrafeEncoder;
        
        // Convert encoder ticks to inches
        double leftDistance = deltaLeft * INCHES_PER_ENCODER_TICK;
        double rightDistance = deltaRight * INCHES_PER_ENCODER_TICK;
        double strafeDistance = deltaStrafe * INCHES_PER_ENCODER_TICK;
        
        // Calculate forward movement and heading change
        double forwardDistance = (leftDistance + rightDistance) / 2.0;
        double headingChange = (rightDistance - leftDistance) / ROBOT_WIDTH * 57.2958; // Convert to degrees
        
        // Update robot heading
        robotHeading += headingChange;
        if (robotHeading > 180) robotHeading -= 360;
        if (robotHeading < -180) robotHeading += 360;
        
        // Update robot position using heading
        double headingRad = Math.toRadians(robotHeading);
        robotX += forwardDistance * Math.cos(headingRad) - strafeDistance * Math.sin(headingRad);
        robotY += forwardDistance * Math.sin(headingRad) + strafeDistance * Math.cos(headingRad);
        
        // Keep robot position within field bounds
        robotX = Math.max(0, Math.min(FIELD_WIDTH, robotX));
        robotY = Math.max(0, Math.min(FIELD_LENGTH, robotY));
        
        // Update last encoder values
        lastLeftEncoder = currentLeftEncoder;
        lastRightEncoder = currentRightEncoder;
        lastStrafeEncoder = currentStrafeEncoder;
    }
    
    private String getFieldZone() {
        // Determine which zone of the field the robot is in
        if (robotY < TILE_SIZE) {
            return "Red Alliance Zone";
        } else if (robotY > FIELD_LENGTH - TILE_SIZE) {
            return "Blue Alliance Zone";
        } else if (robotX < TILE_SIZE) {
            return "Left Side";
        } else if (robotX > FIELD_WIDTH - TILE_SIZE) {
            return "Right Side";
        } else if (Math.abs(robotX - CENTER_FIELD_X) < TILE_SIZE && 
                   Math.abs(robotY - CENTER_FIELD_Y) < TILE_SIZE) {
            return "Center Field";
        } else {
            return "Mid Field";
        }
    }
    
    private void driveToPosition(double targetX, double targetY, double maxPower) {
        // Calculate distance and angle to target
        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        
        // If very close, stop
        if (distance < 2.0) {
            stopDriveMotors();
            return;
        }
        
        // Simple robot-relative movement (not field-relative for testing)
        double forwardPower = Math.max(0.15, Math.min(maxPower, distance * 0.08)); // Minimum 15% power
        double strafePower = Math.max(-maxPower, Math.min(maxPower, deltaX * 0.08));
        
        // Only use Y component for forward/backward
        if (deltaY > 2.0) {
            forwardPower = Math.max(0.15, Math.min(maxPower, deltaY * 0.08));
        } else if (deltaY < -2.0) {
            forwardPower = -Math.max(0.15, Math.min(maxPower, Math.abs(deltaY) * 0.08));
        } else {
            forwardPower = 0;
        }
        
        // Use X component for strafe
        if (Math.abs(deltaX) > 2.0) {
            strafePower = Math.max(-maxPower, Math.min(maxPower, deltaX * 0.08));
        } else {
            strafePower = 0;
        }
        
        setMecanumPower(forwardPower, strafePower, 0, 0); // No turning for simplicity
        
        telemetry.addData("🧪 DRIVE", "Forward: %.2f, Strafe: %.2f", forwardPower, strafePower);
        telemetry.addData("Target", "X: %.1f, Y: %.1f", targetX, targetY);
        telemetry.addData("Distance to Target", "%.1f inches", distance);
        telemetry.addData("Delta", "X: %.1f, Y: %.1f", deltaX, deltaY);
        
        // COORDINATE SYSTEM MOVEMENT DEBUGGING
        telemetry.addData("🎯 Target From Center", "X: %+.1f, Y: %+.1f", targetX - 72.0, targetY - 72.0);
        telemetry.addData("🤖 Robot From Center", "X: %+.1f, Y: %+.1f", robotX - 72.0, robotY - 72.0);
        telemetry.addData("🔄 Movement Needed", "X: %+.1f, Y: %+.1f", deltaX, deltaY);
        
        // Explain what the robot should be doing
        String movementExplanation = "";
        if (Math.abs(deltaY) > 2.0) {
            movementExplanation += (deltaY > 0) ? "Moving FORWARD (North) " : "Moving BACKWARD (South) ";
        }
        if (Math.abs(deltaX) > 2.0) {
            movementExplanation += (deltaX > 0) ? "Strafing RIGHT (East)" : "Strafing LEFT (West)";
        }
        if (!movementExplanation.isEmpty()) {
            telemetry.addData("🏃 Robot Action", movementExplanation.trim());
        }
    }
    
    private boolean isAtPosition(double targetX, double targetY, double tolerance) {
        double distance = Math.sqrt(Math.pow(targetX - robotX, 2) + Math.pow(targetY - robotY, 2));
        return distance < tolerance;
    }
    
    private void initializeFieldPosition() {
        // COORDINATE TEST MODE - Override starting position
        if (ENABLE_COORDINATE_TEST) {
            robotX = 72.0;  // Start at CENTER field X
            robotY = 72.0;  // Start at CENTER field Y
            robotHeading = 0.0;
            telemetry.addData("🧪 COORDINATE TEST MODE", "ENABLED - CENTER-BASED MOVEMENT TEST");
            telemetry.addData("Test Starting Position", "CENTER (%.1f, %.1f)", robotX, robotY);
            telemetry.addData("Test Pattern", "Center→TopLeft→Center→TopRight→Center→BottomLeft→Center→BottomRight→Center");
            telemetry.addData("Test Waypoints", "%d total waypoints", TEST_WAYPOINTS.length);
            telemetry.addData("⚠️ CALIBRATION NEEDED", "Check INCHES_PER_ENCODER_TICK value!");
        } else {
            // Normal starting position
            robotX = BLUE_ROBOT_START_X;  // Use the configured blue robot start position
            robotY = BLUE_ROBOT_START_Y;  // Use the configured blue robot start position
            robotHeading = 0.0;          // Facing forward (toward opposite alliance)
        }
        
        // Reset encoder tracking
        lastLeftEncoder = leftFront.getCurrentPosition();
        lastRightEncoder = rightFront.getCurrentPosition();
        lastStrafeEncoder = leftBack.getCurrentPosition();
        
        telemetry.addData("Starting Position", "X: %.1f, Y: %.1f", robotX, robotY);
        telemetry.addData("Starting Heading", "%.1f degrees", robotHeading);
    }
    
    // COORDINATE TEST FUNCTION - TEMPORARY FOR VERIFICATION
    private void executeCoordinateTest() {
        // Check if we've completed all waypoints
        if (currentWaypointIndex >= TEST_WAYPOINTS.length) {
            stopDriveMotors();
            telemetry.addData("🧪 COORDINATE TEST", "Navigation Complete!");
            telemetry.addData("Final Position", "X: %.1f, Y: %.1f", robotX, robotY);
            telemetry.addData("Next Phase", "Moving to shooting phase");
            
            // Transition to shooting phase
            currentState = AutoState.COORDINATE_TEST_SHOOT;
            stageTimer.reset();
            return;
        }
        
        double targetX = TEST_WAYPOINTS[currentWaypointIndex][0];
        double targetY = TEST_WAYPOINTS[currentWaypointIndex][1];
        
        // If currently pausing at a waypoint
        if (isPausing) {
            stopDriveMotors();
            
            if (pauseTimer.seconds() >= TEST_PAUSE_TIME) {
                // Pause complete, move to next waypoint
                isPausing = false;
                currentWaypointIndex++;
                waypointReached = false;
                
                telemetry.addData("🧪 Waypoint %d", "COMPLETE - Moving to next", currentWaypointIndex);
            } else {
                // Still pausing
                double remainingTime = TEST_PAUSE_TIME - pauseTimer.seconds();
                telemetry.addData("🧪 Pausing", "%.1f seconds remaining", remainingTime);
                telemetry.addData("Current Waypoint", "%d: (%.1f, %.1f)", 
                                currentWaypointIndex + 1, targetX, targetY);
            }
            return;
        }
        
        // Check if we've reached the current waypoint
        if (!waypointReached && isAtPosition(targetX, targetY, POSITION_TOLERANCE)) {
            // Reached waypoint - start pause
            waypointReached = true;
            isPausing = true;
            pauseTimer.reset();
            stopDriveMotors();
            
            telemetry.addData("🧪 REACHED", "Waypoint %d: (%.1f, %.1f)", 
                            currentWaypointIndex + 1, targetX, targetY);
            telemetry.addData("Starting Pause", "%.1f seconds", TEST_PAUSE_TIME);
        } else if (!waypointReached) {
            // Navigate to current waypoint
            driveToPosition(targetX, targetY, TEST_DRIVE_SPEED);
            
            double distance = Math.sqrt(Math.pow(targetX - robotX, 2) + Math.pow(targetY - robotY, 2));
            telemetry.addData("🧪 NAVIGATING", "To waypoint %d", currentWaypointIndex + 1);
            telemetry.addData("Target", "(%.1f, %.1f)", targetX, targetY);
            telemetry.addData("Current", "(%.1f, %.1f)", robotX, robotY);
            telemetry.addData("Distance", "%.1f inches", distance);
            telemetry.addData("Progress", "%d / %d waypoints", currentWaypointIndex + 1, TEST_WAYPOINTS.length);
            
            // ADD COORDINATE SYSTEM DEBUGGING
            String targetDescription = getWaypointDescription(currentWaypointIndex);
            double deltaX = targetX - 72.0; // Offset from center
            double deltaY = targetY - 72.0; // Offset from center
            telemetry.addData("📍 Target Location", targetDescription);
            telemetry.addData("📏 From Center", "X: %+.1f, Y: %+.1f", deltaX, deltaY);
            telemetry.addData("🧭 Movement Direction", getMovementDirection(deltaX, deltaY));
        }
        
        // Display test progress
        telemetry.addData("🧪 TEST MODE", "Coordinate System Verification");
        telemetry.addData("Test Speed", "%.1f power", TEST_DRIVE_SPEED);
        telemetry.addData("Position Tolerance", "%.1f inches", POSITION_TOLERANCE);
        
        // ENHANCED COORDINATE SYSTEM DEBUGGING
        telemetry.addData("📐 COORDINATE SYSTEM", "Field Center = (72, 72)");
        telemetry.addData("📍 Current From Center", "X: %+.1f, Y: %+.1f", robotX - 72.0, robotY - 72.0);
        
        // Current location description
        String currentLocation = "";
        double currentDeltaX = robotX - 72.0;
        double currentDeltaY = robotY - 72.0;
        if (Math.abs(currentDeltaX) < 4.0 && Math.abs(currentDeltaY) < 4.0) {
            currentLocation = "AT CENTER";
        } else {
            currentLocation = getMovementDirection(currentDeltaX, currentDeltaY);
        }
        telemetry.addData("📍 Current Zone", currentLocation);
        
        // Show upcoming waypoints
        if (currentWaypointIndex + 1 < TEST_WAYPOINTS.length) {
            double nextX = TEST_WAYPOINTS[currentWaypointIndex + 1][0];
            double nextY = TEST_WAYPOINTS[currentWaypointIndex + 1][1];
            String nextDescription = getWaypointDescription(currentWaypointIndex + 1);
            telemetry.addData("⏭️ Next Waypoint", "%d: %s", currentWaypointIndex + 2, nextDescription);
            telemetry.addData("⏭️ Next Coordinates", "(%.1f, %.1f)", nextX, nextY);
        }
    }
    
    // BALL COLLECTION PHASE FOR COORDINATE TEST
    private void executeCoordinateTestCollect() {
        if (collectionTimer.seconds() == 0) {
            // Start collection systems
            intake.setPower(INTAKE_POWER);
            conveyor.setPower(CONVEYOR_POWER);
            
            // Start indexor rotation to collect balls
            int currentPosition = indexor.getCurrentPosition();
            indexor.setTargetPosition(currentPosition + INDEXOR_TICKS);
            indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            indexor.setPower(0.3);
            
            collectionTimer.reset();
            telemetry.addData("🧪 COLLECTION", "Starting ball collection");
        }
        
        // Simulate ball detection - in real implementation, use sensors
        // For now, we'll collect for a set time or until target reached
        if (collectionTimer.seconds() > 2.0 && testBallsCollected < TEST_TARGET_BALLS) {
            testBallsCollected++;
            
            if (testBallsCollected < TEST_TARGET_BALLS) {
                // Continue indexor for next ball
                int currentPosition = indexor.getCurrentPosition();
                indexor.setTargetPosition(currentPosition + INDEXOR_TICKS);
                indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                indexor.setPower(0.3);
                collectionTimer.reset();
            }
        }
        
        // Check if collection is complete
        if (testBallsCollected >= TEST_TARGET_BALLS || collectionTimer.seconds() > BALL_COLLECTION_TIMEOUT) {
            // Stop collection systems
            intake.setPower(0);
            conveyor.setPower(0);
            indexor.setPower(0);
            
            telemetry.addData("🧪 COLLECTION", "Complete! Collected %d balls", testBallsCollected);
            telemetry.addData("Starting", "Coordinate navigation test");
            
            // Move to coordinate navigation
            currentState = AutoState.COORDINATE_TEST;
            stageTimer.reset();
        } else {
            // Display collection progress
            telemetry.addData("🧪 COLLECTING", "Balls: %d / %d", testBallsCollected, TEST_TARGET_BALLS);
            telemetry.addData("Collection Time", "%.1f seconds", collectionTimer.seconds());
            telemetry.addData("Systems Running", "Intake, Conveyor, Indexor");
        }
    }
    
    // SHOOTING PHASE FOR COORDINATE TEST
    private void executeCoordinateTestShoot() {
        if (stageTimer.seconds() == 0) {
            telemetry.addData("🧪 SHOOTING PHASE", "Moving to center and preparing shooter");
            stageTimer.reset();
        }
        
        // Move to center field for shooting
        if (!isAtPosition(CENTER_FIELD_X, CENTER_FIELD_Y, POSITION_TOLERANCE)) {
            driveToPosition(CENTER_FIELD_X, CENTER_FIELD_Y, TEST_DRIVE_SPEED);
            telemetry.addData("🧪 POSITIONING", "Moving to center field");
            telemetry.addData("Target", "(%.1f, %.1f)", CENTER_FIELD_X, CENTER_FIELD_Y);
            telemetry.addData("Current", "(%.1f, %.1f)", robotX, robotY);
            return;
        }
        
        // At center field - prepare shooter
        stopDriveMotors();
        
        if (!shooterReady) {
            // Start shooter systems
            shooter.setVelocity(SHOOTER_TARGET_VELOCITY);
            shooterServo.setPower(SHOOTER_SERVO_POWER);
            speedLight.setPosition(LIGHT_GREEN_POSITION);
            
            // Check if shooter is ready (80% of target speed)
            double currentVelocity = Math.abs(shooter.getVelocity());
            double speedPercentage = currentVelocity / SHOOTER_TARGET_VELOCITY;
            
            if (speedPercentage >= SHOOTER_SPEED_THRESHOLD) {
                shooterReady = true;
                telemetry.addData("🧪 SHOOTER", "READY! Starting fire sequence");
            } else {
                telemetry.addData("🧪 SHOOTER", "Spinning up: %.1f%%", speedPercentage * 100);
                telemetry.addData("Current Speed", "%.0f / %.0f ticks/sec", currentVelocity, SHOOTER_TARGET_VELOCITY);
            }
        } else {
            // Shooter ready - execute firing sequence
            if (stageTimer.seconds() < 1.0) {
                // Start indexor and conveyor for firing
                conveyor.setPower(CONVEYOR_POWER);
                int currentPosition = indexor.getCurrentPosition();
                indexor.setTargetPosition(currentPosition + INDEXOR_TICKS);
                indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                indexor.setPower(0.3);
                
                telemetry.addData("🧪 FIRING", "Feeding ball to shooter");
                
            } else if (stageTimer.seconds() < 2.0) {
                // Fire trigger
                triggerServo.setPosition(TRIGGER_HOME);
                telemetry.addData("🧪 FIRING", "SHOT FIRED!");
                
            } else if (stageTimer.seconds() < 3.0) {
                // Reset trigger
                triggerServo.setPosition(TRIGGER_FIRE);
                telemetry.addData("🧪 FIRING", "Resetting trigger");
                
            } else {
                // Complete test
                stopAllMotors();
                speedLight.setPosition(LIGHT_OFF_POSITION);
                
                telemetry.addData("🧪 COORDINATE TEST", "FULLY COMPLETED!");
                telemetry.addData("Ball Collection", "%d balls collected", testBallsCollected);
                telemetry.addData("Navigation", "All %d waypoints completed", TEST_WAYPOINTS.length);
                telemetry.addData("Shooting", "Fire sequence executed");
                telemetry.addData("Final Position", "Center field (%.1f, %.1f)", CENTER_FIELD_X, CENTER_FIELD_Y);
                
                currentState = AutoState.COMPLETE;
            }
        }
    }
    
    // COORDINATE SYSTEM DEBUGGING HELPERS
    private String getWaypointDescription(int waypointIndex) {
        switch(waypointIndex) {
            case 0: return "CENTER (Starting Point)";
            case 1: return "TOP LEFT (-36X, +36Y from center)";
            case 2: return "CENTER (Return from Top Left)";
            case 3: return "TOP RIGHT (+36X, +36Y from center)";
            case 4: return "CENTER (Return from Top Right)";
            case 5: return "BOTTOM LEFT (-36X, -36Y from center)";
            case 6: return "CENTER (Return from Bottom Left)";
            case 7: return "BOTTOM RIGHT (+36X, -36Y from center)";
            case 8: return "CENTER (Final Return)";
            default: return "Unknown Waypoint";
        }
    }
    
    private String getMovementDirection(double deltaX, double deltaY) {
        if (Math.abs(deltaX) < 2.0 && Math.abs(deltaY) < 2.0) {
            return "AT CENTER";
        }
        
        String direction = "";
        
        // Determine Y direction (North/South)
        if (deltaY > 2.0) {
            direction += "NORTH ";
        } else if (deltaY < -2.0) {
            direction += "SOUTH ";
        }
        
        // Determine X direction (East/West)
        if (deltaX > 2.0) {
            direction += "EAST";
        } else if (deltaX < -2.0) {
            direction += "WEST";
        }
        
        return direction.trim();
    }
}