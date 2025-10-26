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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.WebcamName;
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
    public static final double SHOOTER_SERVO_POWER = 1.0;
    public static final double TRIGGER_SERVO_MIN_POSITION = 0.0;      // 0 degrees
    public static final double TRIGGER_SERVO_MAX_POSITION = 0.333;    // 60 degrees
    public static final double LIGHT_OFF_POSITION = 0.0;
    public static final double LIGHT_GREEN_POSITION = 0.5;
    
    // Detection thresholds
    public static final double BALL_DETECTION_DISTANCE = 12.0;  // inches - distance to detect ball
    public static final double BALL_PICKUP_DISTANCE = 4.0;      // inches - distance to be over ball
    public static final int TARGET_TAG_ID = 20;                 // Blue AprilTag ID
    public static final double TAG_APPROACH_DISTANCE = 24.0;    // inches - stop 2 feet from tag
    public static final double TAG_ALIGNMENT_TOLERANCE = 3.0;   // inches - alignment tolerance
    
    // Timing constants
    public static final double SCAN_TIMEOUT = 10.0;         // seconds - max time to scan for objects
    public static final double PICKUP_DURATION = 3.0;       // seconds - time to run intake for pickup
    public static final double SHOOTER_SPINUP_TIME = 2.0;   // seconds - time for shooter to reach speed
    public static final double FIRING_SEQUENCE_TIME = 1.5;  // seconds - time for firing sequence
    
    // Shooter velocity control
    public static final double SHOOTER_TARGET_VELOCITY = 1600; // ticks/sec
    
    // Autonomous state tracking
    private enum AutoState {
        INIT,
        SCAN_FOR_BALL,
        APPROACH_BALL,
        PICKUP_BALL,
        SCAN_FOR_TAG,
        APPROACH_TAG,
        PREPARE_SHOOTER,
        FIRE_SEQUENCE,
        COMPLETE
    }
    
    private AutoState currentState = AutoState.INIT;
    private boolean ballDetected = false;
    private boolean tagDetected = false;
    private AprilTagDetection targetTag = null;
    
    @Override
    public void runOpMode() {
        // Initialize all hardware
        initializeMotors();
        initializeAprilTag();
        initializeSensors();
        
        // Display initialization status
        telemetry.addData("Status", "AutoOpDECODE Initialized");
        telemetry.addData("Mission", "1. Find ball → 2. Pickup → 3. Find AprilTag → 4. Fire");
        telemetry.addData("Target", "Blue AprilTag ID %d", TARGET_TAG_ID);
        telemetry.addData("Ready", "Press PLAY to start autonomous");
        telemetry.update();
        
        waitForStart();
        runtime.reset();
        
        // Main autonomous loop
        while (opModeIsActive()) {
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
        shooterServo.setDirection(DcMotorSimple.Direction.REVERSE);
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
        triggerServo.setPosition(TRIGGER_SERVO_MIN_POSITION);
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
        // Create the AprilTag processor - same as TeleOp
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagGameDatabase.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Create the vision portal
        VisionPortal.Builder builder = new VisionPortal.Builder();
        
        // Set the camera
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        
        // Choose a camera resolution optimized for autonomous
        builder.setCameraResolution(new Size(640, 480));
        
        // Enable the RC preview for debugging
        builder.enableLiveView(true);
        
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
                currentState = AutoState.SCAN_FOR_BALL;
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
                
            case COMPLETE:
                stopAllMotors();
                break;
        }
    }
    
    private void scanForBall() {
        if (stageTimer.seconds() == 0) {
            telemetry.addData("Stage", "Scanning for ball...");
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
            telemetry.addData("Ball", "DETECTED! Approaching...");
        }
        
        // Timeout if no ball found
        if (stageTimer.seconds() > SCAN_TIMEOUT) {
            telemetry.addData("Warning", "Ball scan timeout - proceeding to AprilTag");
            stopDriveMotors();
            currentState = AutoState.SCAN_FOR_TAG;
            stageTimer.reset();
        }
    }
    
    private void approachBall() {
        if (stageTimer.seconds() == 0) {
            telemetry.addData("Stage", "Approaching ball...");
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
        
        // Timeout safety
        if (stageTimer.seconds() > SCAN_TIMEOUT) {
            stopDriveMotors();
            currentState = AutoState.SCAN_FOR_TAG;
            stageTimer.reset();
        }
    }
    
    private void pickupBall() {
        if (stageTimer.seconds() == 0) {
            telemetry.addData("Stage", "Picking up ball...");
            // Start intake, conveyor, and indexor
            intake.setPower(INTAKE_POWER);
            conveyor.setPower(CONVEYOR_POWER);
            
            // Move indexor to pickup position
            int currentPosition = indexor.getCurrentPosition();
            indexor.setTargetPosition(currentPosition + INDEXOR_TICKS);
            indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            indexor.setPower(0.8);
            
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
            
            telemetry.addData("Ball", "Pickup complete!");
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
            telemetry.addData("Stage", "Preparing shooter...");
            // Start shooter and shooter servo
            shooter.setVelocity(SHOOTER_TARGET_VELOCITY);
            shooterServo.setPower(SHOOTER_SERVO_POWER);
            speedLight.setPosition(LIGHT_GREEN_POSITION);
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
            telemetry.addData("Stage", "FIRING!");
            stageTimer.reset();
        }
        
        double sequenceTime = stageTimer.seconds();
        
        if (sequenceTime < 0.5) {
            // Start indexor and conveyor to feed ball
            conveyor.setPower(CONVEYOR_POWER);
            int currentPosition = indexor.getCurrentPosition();
            indexor.setTargetPosition(currentPosition + INDEXOR_TICKS);
            indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            indexor.setPower(0.8);
            
        } else if (sequenceTime < 1.0) {
            // Fire trigger servo
            triggerServo.setPosition(TRIGGER_SERVO_MAX_POSITION);
            telemetry.addData("Trigger", "FIRED!");
            
        } else if (sequenceTime < FIRING_SEQUENCE_TIME) {
            // Reset trigger servo
            triggerServo.setPosition(TRIGGER_SERVO_MIN_POSITION);
            
        } else {
            // Complete firing sequence
            stopAllMotors();
            speedLight.setPosition(LIGHT_OFF_POSITION);
            telemetry.addData("Mission", "COMPLETE!");
            currentState = AutoState.COMPLETE;
        }
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
}