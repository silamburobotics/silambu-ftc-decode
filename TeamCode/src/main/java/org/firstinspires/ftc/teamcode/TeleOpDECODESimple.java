package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
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

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "TeleOpDECODESimple", group = "TeleOp")
public class TeleOpDECODESimple extends LinearOpMode {
    
    // Declare motors
    private DcMotorEx indexor;
    private DcMotorEx intake;
    private DcMotorEx conveyor;
    private DcMotorEx shooter;
    
    // Declare servos
    private CRServo shooterServo;
    private Servo speedLight;
    private Servo triggerServo;
    
    // Declare mecanum drive motors
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    
    // Variables to track button states
    private boolean previousA = false;  // Gamepad1 A button (intake)
    
    // Gamepad2 button states (operator controls)
    private boolean previousX2 = false; // Gamepad2 X button (indexor)
    private boolean previousY2 = false; // Gamepad2 Y button (shooter)
    private boolean previousB2 = false; // Gamepad2 B button (trigger)
    
    // Shooter state tracking
    private boolean shooterIntentionallyRunning = false;
    
    // Trigger servo management
    private boolean triggerHeld = false; // Track if trigger is currently being held down
    
    // AprilTag detection
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private boolean isAlignedToTag = false; // Track if robot is aligned to AprilTag
    private boolean aprilTagDistanceAvailable = false; // Track if AprilTag distance is available
    private int currentDetectedTagId = -1; // Track which tag is currently detected (20 or 24)
    
    // Indexor stuck detection variables
    private ElapsedTime indexorTimer = new ElapsedTime();
    private int lastIndexorPosition = 0;
    private boolean indexorStuckDetectionActive = false;
    
    // Indexor position control variables
    private boolean indexorRunningToPosition = false;
    
    // Motor power settings
    public static final double INTAKE_POWER = 0.8;
    public static final double CONVEYOR_POWER = 1.0;
    public static final double AUTO_INDEXOR_POWER = 0.1;      // Power for automatic indexor movement
    public static final double SHOOTER_POWER = 1.0;
    public static final double SHOOTER_SERVO_POWER = 1.0;     // Positive for forward direction
    
    // Indexor position settings
    public static final int INDEXOR_TICKS = 179;              // goBILDA 312 RPM motor: 120 degrees = 179 ticks
    
    // Shooter velocity control (ticks per second)
    public static double SHOOTER_TARGET_VELOCITY = 1300;      // Range: 1200-1800 ticks/sec
    public static final double SHOOTER_SPEED_THRESHOLD = 0.95; // 95% of target speed
    public static final double SHOOTER_TICKS_PER_REVOLUTION = 1020.0; // goBILDA 435 RPM motor
    
    // Speed light control settings (using servo positions for LED control)
    public static final double LIGHT_OFF_POSITION = 0.0;      // Servo position for light off
    public static final double LIGHT_YELLOW_POSITION = 0.33;  // Servo position for yellow light (close range)
    public static final double LIGHT_GREEN_POSITION = 0.5;    // Servo position for green light (long range)
    public static final double LIGHT_WHITE_POSITION = 1.0;    // Servo position for white light
    
    // Trigger servo positions
    public static final double TRIGGER_FIRE = 0.15;     // 27 degrees (27/180 = 0.15) - FIRE POSITION (compact)
    public static final double TRIGGER_HOME = 0.76;     // 137 degrees (137/180 = 0.76) - HOME/SAFE POSITION (retracted)
    
    // Stuck detection settings
    public static final double INDEXOR_STUCK_TIMEOUT = 3.0;    // seconds - time before considering indexor stuck
    public static final int INDEXOR_PROGRESS_THRESHOLD = 20;   // minimum ticks of progress required
    
    // Mecanum drive settings
    public static final double DRIVE_SPEED_MULTIPLIER = 0.8;  // Max drive speed (0.0 to 1.0)
    public static final double STRAFE_SPEED_MULTIPLIER = 0.8; // Max strafe speed (0.0 to 1.0)
    public static final double TURN_SPEED_MULTIPLIER = 0.6;   // Max turn speed (0.0 to 1.0)
    
    // AprilTag detection settings
    public static final int BLUE_TAG_ID = 20; // Blue AprilTag ID
    public static final int RED_TAG_ID = 24; // Red AprilTag ID
    public static final double ALIGNMENT_TOLERANCE = 5.0; // degrees
    public static final double MIN_TAG_AREA = 100.0; // Minimum tag area for reliable detection
    
    // Distance-based shooter velocity settings
    public static final double CLOSE_RANGE_DISTANCE = 90.0;   // inches - threshold for close vs long range
    public static final double CLOSE_RANGE_VELOCITY = 1250;   // ticks/sec for close range shooting
    public static final double LONG_RANGE_VELOCITY = 1600;    // ticks/sec for long range shooting
    public static final double DEFAULT_VELOCITY = 1250;       // ticks/sec when no AprilTag detected
    
    @Override
    public void runOpMode() {
        // Initialize motors
        initializeMotors();
        
        // Initialize AprilTag detection
        initializeAprilTag();
        
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "DECODE Simple + AprilTag - Initialized");
        telemetry.addData("=== GAMEPAD 1 (DRIVER) ===", "");
        telemetry.addData("A Button", "Intake + Conveyor + Indexor");
        telemetry.addData("Left Stick", "Drive/Strafe");
        telemetry.addData("Right Stick X", "Turn");
        telemetry.addData("=== GAMEPAD 2 (OPERATOR) ===", "");
        telemetry.addData("X Button", "Indexor + Conveyor (%d ticks)", INDEXOR_TICKS);
        telemetry.addData("Y Button", "Shooter + Servo (Distance-Adaptive)");
        telemetry.addData("B Button", "Hold for Fire / Release for Home");
        telemetry.addData("", "");
        telemetry.addData("AprilTag Targets", "🔵 Blue ID %d | 🔴 Red ID %d", BLUE_TAG_ID, RED_TAG_ID);
        telemetry.addData("Close Range", "<%.0f inches = %.0f ticks/sec 🟡", CLOSE_RANGE_DISTANCE, CLOSE_RANGE_VELOCITY);
        telemetry.addData("Long Range", "≥%.0f inches = %.0f ticks/sec 🟢", CLOSE_RANGE_DISTANCE, LONG_RANGE_VELOCITY);
        telemetry.addData("Default (No Tag)", "%.0f ticks/sec ⚪", DEFAULT_VELOCITY);
        telemetry.addData("Speed Light", "🟡 Yellow=Close 🟢 Green=Long ⚪ White=Medium");
        telemetry.addData("Drive Speed", "%.0f%% max", DRIVE_SPEED_MULTIPLIER * 100);
        telemetry.addData("Strafe Speed", "%.0f%% max", STRAFE_SPEED_MULTIPLIER * 100);
        telemetry.addData("Turn Speed", "%.0f%% max", TURN_SPEED_MULTIPLIER * 100);
        telemetry.addData("Note", "Driver/Operator Split Controls");
        telemetry.update();
        
        waitForStart();
        
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            handleControllerInputs();
            handleMecanumDrive();
            handleAprilTagAlignment(); // Monitor AprilTag and adjust shooter velocity
            checkIndexorStuck();
            handleTriggerHoldAndRelease(); // Handle hold-and-release trigger control
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
        conveyor = hardwareMap.get(DcMotorEx.class, "conveyor");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        
        // Initialize servos
        shooterServo = hardwareMap.get(CRServo.class, "shooterServo");
        speedLight = hardwareMap.get(Servo.class, "speedLight");
        triggerServo = hardwareMap.get(Servo.class, "triggerServo");
        
        // Initialize mecanum drive motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        
        // Set motor directions
        indexor.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        conveyor.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        
        // Set servo directions
        shooterServo.setDirection(DcMotorSimple.Direction.REVERSE);
        speedLight.setDirection(Servo.Direction.FORWARD);
        triggerServo.setDirection(Servo.Direction.FORWARD);
        
        // Initialize speed light to off position
        speedLight.setPosition(LIGHT_OFF_POSITION);
        
        // Initialize trigger servo to home (safe) position
        triggerServo.setPosition(TRIGGER_HOME);
        
        // Set mecanum drive motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        
        // Set zero power behavior
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        // Set mecanum drive motor zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Reset encoders
        indexor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set shooter mode
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set mecanum drive motors to run without encoders for teleop
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set indexor to use encoder
        indexor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    private void initializeAprilTag() {
        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                // Arducam OV9281 optimized lens intrinsics for 1280x720 resolution
                .setLensIntrinsics(1156.544, 1156.544, 640.0, 360.0) // fx, fy, cx, cy for 1280x720
                .build();

        // Create the vision portal with Arducam OV9281 specific settings
        VisionPortal.Builder builder = new VisionPortal.Builder();
        
        // Set the camera - Change this to match your hardware config name
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")); // Update this name if different
        
        // Arducam OV9281 optimal resolution settings
        builder.setCameraResolution(new android.util.Size(1280, 720)); // Best for AprilTag detection
        
        // Arducam OV9281 specific settings for optimal performance
        builder.enableLiveView(true); // Enable for debugging, disable for competition performance
        builder.setAutoStopLiveView(false); // Keep live view running
        
        // Set AprilTag processor
        builder.addProcessor(aprilTag);
        
        // Build the Vision Portal
        visionPortal = builder.build();
        
        // Configure Arducam OV9281 camera settings for optimal AprilTag detection
        configureArducamOV9281();
        
        telemetry.addData("Camera", "Arducam OV9281 Global Shutter");
        telemetry.addData("Resolution", "1280x720 (optimal for AprilTags)");
        telemetry.addData("AprilTag Vision", "Initialized with TAG_36h11 family");
        telemetry.addData("Target Tags", "🔵 Blue ID %d | 🔴 Red ID %d", BLUE_TAG_ID, RED_TAG_ID);
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
            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure(15, TimeUnit.MILLISECONDS); // Adjust based on your lighting
            telemetry.addData("Exposure", "Set to 15ms (manual mode)");
        }
        
        if (gainControl != null) {
            gainControl.setGain(50); // Range typically 0-255, adjust as needed
            telemetry.addData("Gain", "Set to 50");
        }
        
        if (focusControl != null) {
            focusControl.setMode(FocusControl.Mode.Fixed);
            focusControl.setFocusLength(240.0); // Focus at distance for tags
            telemetry.addData("Focus", "Set to fixed 240.0");
        }
        
        if (whiteBalanceControl != null) {
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
        aprilTagDistanceAvailable = false;
        currentDetectedTagId = -1;
        
        // Look for either blue (20) or red (24) target tags
        for (AprilTagDetection detection : currentDetections) {
            if (detection != null && detection.ftcPose != null && 
                (detection.id == BLUE_TAG_ID || detection.id == RED_TAG_ID)) {
                // Valid target tag found with pose data - distance is available!
                aprilTagDistanceAvailable = true;
                currentDetectedTagId = detection.id;
                
                // Get distance and pose information
                double xOffset = detection.ftcPose.x;
                double yOffset = detection.ftcPose.y;
                double headingError = detection.ftcPose.yaw;
                double distance = detection.ftcPose.range; // Distance in inches
                
                // Determine distance category and set shooter velocity
                String distanceCategory;
                String distanceAdvice;
                if (distance < CLOSE_RANGE_DISTANCE) {
                    distanceCategory = "🔵 CLOSE RANGE";
                    distanceAdvice = "Within optimal range";
                    SHOOTER_TARGET_VELOCITY = CLOSE_RANGE_VELOCITY;
                } else {
                    distanceCategory = "🔴 LONG RANGE";
                    distanceAdvice = "Beyond " + (int)CLOSE_RANGE_DISTANCE + " inches";
                    SHOOTER_TARGET_VELOCITY = LONG_RANGE_VELOCITY;
                }
                
                // Update shooter velocity if shooter is running
                if (shooterIntentionallyRunning && shooter != null) {
                    shooter.setVelocity(SHOOTER_TARGET_VELOCITY);
                }
                
                // Check if we're aligned within tolerance
                boolean xAligned = Math.abs(xOffset) < ALIGNMENT_TOLERANCE;
                boolean yAligned = Math.abs(yOffset) < ALIGNMENT_TOLERANCE;
                boolean headingAligned = Math.abs(headingError) < ALIGNMENT_TOLERANCE;
                
                isAlignedToTag = xAligned && yAligned && headingAligned;
                
                return; // Found our tag, no need to check others
            }
        }
        
        // If we get here, the target tag wasn't found - use default velocity
        SHOOTER_TARGET_VELOCITY = DEFAULT_VELOCITY;
        
        // Update shooter velocity if shooter is running
        if (shooterIntentionallyRunning && shooter != null) {
            shooter.setVelocity(SHOOTER_TARGET_VELOCITY);
        }
    }
    
    private void handleControllerInputs() {
        // Get current button states for gamepad1 (driver)
        boolean currentA = gamepad1.a;
        
        // Get current button states for gamepad2 (operator)
        boolean currentX2 = gamepad2.x;
        boolean currentY2 = gamepad2.y;
        boolean currentB2 = gamepad2.b;
        
        // Handle A button on gamepad1 - Toggle Intake, Conveyor, and Indexor
        if (currentA && !previousA) {
            toggleIntakeSystem();
        }
        
        // Handle X button on gamepad2 - Run Indexor to Position
        if (currentX2 && !previousX2) {
            runIndexorToPosition();
        }
        
        // Handle Y button on gamepad2 - Toggle Shooter
        if (currentY2 && !previousY2) {
            toggleShooter();
        }
        
        // Handle B button on gamepad2 - Hold/Release Trigger Servo
        handleTriggerButton(currentB2);
        
        // Update previous button states for gamepad1
        previousA = currentA;
        
        // Update previous button states for gamepad2
        previousX2 = currentX2;
        previousY2 = currentY2;
        previousB2 = currentB2;
    }
    
    private void toggleIntakeSystem() {
        // Check if any motor is running
        boolean isRunning = (Math.abs(intake.getPower()) > 0.1) || 
                           (Math.abs(conveyor.getPower()) > 0.1);
        
        if (isRunning) {
            // Stop all motors: intake, conveyor, and indexor
            intake.setPower(0);
            conveyor.setPower(0);
            indexor.setPower(0);
            // Reset indexor to BRAKE behavior and stop all detection
            indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            indexor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            indexorStuckDetectionActive = false;
            indexorRunningToPosition = false;
            telemetry.addData("Intake System", "STOPPED");
        } else {
            // Start all motors: intake, conveyor, and indexor
            intake.setPower(INTAKE_POWER);
            conveyor.setPower(CONVEYOR_POWER);
            indexor.setPower(AUTO_INDEXOR_POWER);
            // Reset indexor to BRAKE behavior and start stuck detection
            indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            startIndexorStuckDetection();
            telemetry.addData("Intake System", "RUNNING");
            telemetry.addData("Intake", "%.1f power", INTAKE_POWER);
            telemetry.addData("Conveyor", "%.1f power", CONVEYOR_POWER);
            telemetry.addData("Indexor", "%.1f power", AUTO_INDEXOR_POWER);
        }
        telemetry.update();
    }
    
    private void runIndexorToPosition() {
        // Stop any continuous power mode first
        indexor.setPower(0);
        
        // Start conveyor to help feed balls through the system
        conveyor.setPower(CONVEYOR_POWER);
        
        // Calculate target position
        int currentPosition = indexor.getCurrentPosition();
        int targetPosition = currentPosition + INDEXOR_TICKS;
        
        // Reset motor behavior to BRAKE and set to position mode
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexor.setTargetPosition(targetPosition);
        indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexor.setPower(AUTO_INDEXOR_POWER);
        
        // Start tracking
        indexorRunningToPosition = true;
        startIndexorStuckDetection();
        
        telemetry.addData("Indexor Position", "Moving to: %d (from %d)", targetPosition, currentPosition);
        telemetry.addData("Conveyor", "RUNNING at %.1f power", CONVEYOR_POWER);
        telemetry.addData("Indexor Ticks", "%d", INDEXOR_TICKS);
        telemetry.update();
    }
    
    private void startIndexorStuckDetection() {
        indexorStuckDetectionActive = true;
        indexorTimer.reset();
        lastIndexorPosition = indexor.getCurrentPosition();
    }
    
    private void checkIndexorStuck() {
        // Check if indexor is running to position and completed
        if (indexorRunningToPosition && !indexor.isBusy()) {
            indexorRunningToPosition = false;
            indexorStuckDetectionActive = false;
            indexor.setPower(0);
            
            // Set indexor to FLOAT mode so trigger can push balls through freely
            indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            
            // Stop conveyor when indexor positioning is complete
            conveyor.setPower(0);
            
            telemetry.addData("✅ INDEXOR", "Position reached - FLOATING for free trigger movement");
            telemetry.addData("✅ CONVEYOR", "Stopped");
            return;
        }
        
        if (!indexorStuckDetectionActive || Math.abs(indexor.getPower()) < 0.01) {
            return; // Not monitoring or indexor is not running
        }
        
        int currentPosition = indexor.getCurrentPosition();
        double timeElapsed = indexorTimer.seconds();
        
        // Check if enough time has passed for stuck detection
        if (timeElapsed > INDEXOR_STUCK_TIMEOUT) {
            int positionChange = Math.abs(currentPosition - lastIndexorPosition);
            
            // Check if indexor hasn't moved enough (stuck)
            if (positionChange < INDEXOR_PROGRESS_THRESHOLD) {
                // Indexor is stuck - set to float and stop conveyor
                indexor.setPower(0);
                indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                conveyor.setPower(0);  // Stop conveyor when stuck
                indexorStuckDetectionActive = false;
                indexorRunningToPosition = false;
                
                telemetry.addData("⚠️ INDEXOR STUCK", "Set to FLOAT mode");
                telemetry.addData("⚠️ CONVEYOR", "Stopped due to stuck indexor");
                telemetry.addData("Position Change", "%d ticks in %.1fs", positionChange, timeElapsed);
                telemetry.addData("Action", "Motor floating - press X or A to restart");
            } else {
                // Reset detection timer if making progress
                indexorTimer.reset();
                lastIndexorPosition = currentPosition;
            }
        }
    }
    
    private void toggleShooter() {
        // Check if shooter is currently running
        if (shooterIntentionallyRunning) {
            // Stop shooter and servo
            shooter.setPower(0);
            shooterServo.setPower(0);
            shooterIntentionallyRunning = false;
            
            telemetry.addData("Shooter System", "STOPPED");
        } else {
            // Ensure we have the latest AprilTag-based velocity
            handleAprilTagAlignment();
            
            // Start shooter with distance-adaptive velocity control and servo
            shooter.setVelocity(SHOOTER_TARGET_VELOCITY);
            shooterServo.setPower(SHOOTER_SERVO_POWER);
            shooterIntentionallyRunning = true;
            
            // Provide detailed feedback about velocity selection
            String velocityReason;
            if (aprilTagDistanceAvailable) {
                if (SHOOTER_TARGET_VELOCITY == CLOSE_RANGE_VELOCITY) {
                    velocityReason = String.format("🔵 CLOSE RANGE (%.0f ticks/sec)", CLOSE_RANGE_VELOCITY);
                } else {
                    velocityReason = String.format("🔴 LONG RANGE (%.0f ticks/sec)", LONG_RANGE_VELOCITY);
                }
            } else {
                velocityReason = String.format("🎯 DEFAULT (%.0f ticks/sec) - No AprilTag", DEFAULT_VELOCITY);
            }
            
            telemetry.addData("Shooter System", "RUNNING - Distance Adaptive");
            telemetry.addData("Velocity Mode", velocityReason);
            if (aprilTagDistanceAvailable) {
                String tagColor = (currentDetectedTagId == BLUE_TAG_ID) ? "🔵 Blue" : "🔴 Red";
                telemetry.addData("AprilTag Detected", "%s ID %d", tagColor, currentDetectedTagId);
            } else {
                telemetry.addData("AprilTag Search", "Looking for 🔵 Blue %d | 🔴 Red %d", BLUE_TAG_ID, RED_TAG_ID);
            }
        }
        telemetry.update();
    }
    
    private void updateSpeedLight() {
        if (!shooterIntentionallyRunning) {
            // Shooter is off - speed light should be off
            speedLight.setPosition(LIGHT_OFF_POSITION);
            return;
        }
        
        // Shooter is on - check speed and range
        double currentVelocity = shooter.getVelocity();
        double speedPercentage = currentVelocity / SHOOTER_TARGET_VELOCITY;
        
        if (currentVelocity > 50 && speedPercentage > SHOOTER_SPEED_THRESHOLD) {
            // Speed is good - set color based on range
            if (SHOOTER_TARGET_VELOCITY == CLOSE_RANGE_VELOCITY) {
                // Close range - yellow light
                speedLight.setPosition(LIGHT_YELLOW_POSITION);
            } else if (SHOOTER_TARGET_VELOCITY == LONG_RANGE_VELOCITY) {
                // Long range - green light
                speedLight.setPosition(LIGHT_GREEN_POSITION);
            } else {
                // Default velocity - white light
                speedLight.setPosition(LIGHT_WHITE_POSITION);
            }
        } else if (currentVelocity > 50 && speedPercentage > 0.7) {
            // Speed is medium - white light regardless of range
            speedLight.setPosition(LIGHT_WHITE_POSITION);
        } else {
            // Speed is low - off (no light)
            speedLight.setPosition(LIGHT_OFF_POSITION);
        }
    }
    
    private void handleTriggerButton(boolean buttonPressed) {
        // Check if intake is running - safety first!
        boolean intakeRunning = Math.abs(intake.getPower()) > 0.1;
        
        if (buttonPressed && !intakeRunning) {
            // Button is held down and it's safe - move to FIRE position
            if (!triggerHeld) {
                triggerServo.setPosition(TRIGGER_FIRE);
                triggerHeld = true;
            }
        } else {
            // Button released or intake running - move to HOME position
            if (triggerHeld) {
                triggerServo.setPosition(TRIGGER_HOME);
                triggerHeld = false;
            }
        }
    }
    
    private void handleTriggerHoldAndRelease() {
        // This function handles any additional trigger state management if needed
        // Currently, all logic is handled in handleTriggerButton()
    }
    
    private void handleMecanumDrive() {
        // Get joystick inputs
        double drive = -gamepad1.left_stick_y;  // Forward/backward (reversed: negative for forward - robot facing swapped)
        double strafe = gamepad1.left_stick_x;  // Left/right strafe (normal: positive for right strafe)
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
    
    private void updateTelemetry() {
        // Show current system status
        telemetry.addData("Status", "DECODE Simple + Full System - RUNNING");
        telemetry.addData("", "");
        
        // Show drive values
        telemetry.addData("Drive", "%.2f", -gamepad1.left_stick_y);
        telemetry.addData("Strafe", "%.2f", gamepad1.left_stick_x);
        telemetry.addData("Turn", "%.2f", -gamepad1.right_stick_x);
        telemetry.addData("", "");
        
        // Show motor powers
        telemetry.addData("Intake Power", "%.2f", intake.getPower());
        telemetry.addData("Conveyor Power", "%.2f", conveyor.getPower());
        telemetry.addData("Indexor Power", "%.2f", indexor.getPower());
        telemetry.addData("", "");
        
        // Show shooter status
        if (shooterIntentionallyRunning) {
            telemetry.addData("Shooter", "RUNNING");
            telemetry.addData("Target Velocity", "%.0f ticks/sec", SHOOTER_TARGET_VELOCITY);
            telemetry.addData("Current Velocity", "%.0f ticks/sec", shooter.getVelocity());
            telemetry.addData("Shooter Servo", "%.2f", shooterServo.getPower());
            
            // Show speed light status with color indication
            double lightPosition = speedLight.getPosition();
            String lightStatus;
            if (Math.abs(lightPosition - LIGHT_YELLOW_POSITION) < 0.05) {
                lightStatus = "🟡 YELLOW (Close Range Ready)";
            } else if (Math.abs(lightPosition - LIGHT_GREEN_POSITION) < 0.05) {
                lightStatus = "🟢 GREEN (Long Range Ready)";
            } else if (Math.abs(lightPosition - LIGHT_WHITE_POSITION) < 0.05) {
                lightStatus = "⚪ WHITE (Medium Speed)";
            } else {
                lightStatus = "⚫ OFF (Low Speed)";
            }
            telemetry.addData("Speed Light", "%s (%.2f)", lightStatus, lightPosition);
        } else {
            telemetry.addData("Shooter", "STOPPED");
        }
        telemetry.addData("", "");
        
        // Show AprilTag detection and distance-based velocity
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("👁️ Vision Status", "%d tags detected", currentDetections.size());
        
        if (aprilTagDistanceAvailable) {
            // Find and display our detected tag information
            for (AprilTagDetection detection : currentDetections) {
                if (detection != null && detection.ftcPose != null && 
                    (detection.id == BLUE_TAG_ID || detection.id == RED_TAG_ID)) {
                    double distance = detection.ftcPose.range;
                    String distanceCategory = distance < CLOSE_RANGE_DISTANCE ? "🔵 CLOSE" : "🔴 LONG";
                    double selectedVelocity = distance < CLOSE_RANGE_DISTANCE ? CLOSE_RANGE_VELOCITY : LONG_RANGE_VELOCITY;
                    String tagColor = (detection.id == BLUE_TAG_ID) ? "🔵 Blue" : "🔴 Red";
                    
                    telemetry.addData("🎯 AprilTag", "%s ID %d DETECTED", tagColor, detection.id);
                    telemetry.addData("📏 Distance", "%.1f inches (%s RANGE)", distance, distanceCategory.substring(2));
                    telemetry.addData("🚀 Auto Velocity", "%.0f ticks/sec", selectedVelocity);
                    telemetry.addData("🧭 Position", "X: %.1f, Y: %.1f", detection.ftcPose.x, detection.ftcPose.y);
                    telemetry.addData("📐 Heading", "%.1f degrees", detection.ftcPose.yaw);
                    break;
                }
            }
        } else {
            telemetry.addData("🔍 AprilTag", "Searching for 🔵 Blue %d | 🔴 Red %d", BLUE_TAG_ID, RED_TAG_ID);
            telemetry.addData("🎯 Default Mode", "%.0f ticks/sec (No tag)", DEFAULT_VELOCITY);
            
            // Show any visible tags for debugging
            for (AprilTagDetection detection : currentDetections) {
                if (detection != null) {
                    telemetry.addData("👀 Visible Tag", "ID %d", detection.id);
                }
            }
        }
        telemetry.addData("", "");
        
        // Show trigger status
        double currentTriggerPosition = triggerServo.getPosition();
        boolean triggerInFirePosition = Math.abs(currentTriggerPosition - TRIGGER_FIRE) < 0.05;
        boolean intakeRunning = Math.abs(intake.getPower()) > 0.1;
        
        if (intakeRunning && gamepad2.b) {
            telemetry.addData("Trigger", "🚫 BLOCKED - Intake running");
            telemetry.addData("Safety", "Stop intake first");
        } else if (triggerHeld && triggerInFirePosition) {
            telemetry.addData("Trigger", "🎯 FIRE (%.2f) - Button HELD", currentTriggerPosition);
        } else if (triggerInFirePosition) {
            telemetry.addData("Trigger", "🎯 FIRE (%.2f)", currentTriggerPosition);
        } else {
            telemetry.addData("Trigger", "🏠 HOME (%.2f)", currentTriggerPosition);
        }
        
        if (gamepad2.b) {
            telemetry.addData("B Button", "PRESSED - %s", intakeRunning ? "BLOCKED" : "FIRING");
        } else {
            telemetry.addData("B Button", "Released");
        }
        
        // Show stuck detection status
        if (indexorRunningToPosition) {
            telemetry.addData("Indexor Mode", "POSITION CONTROL");
            telemetry.addData("Current Position", "%d", indexor.getCurrentPosition());
            telemetry.addData("Target Position", "%d", indexor.getTargetPosition());
            telemetry.addData("Is Busy", "%s", indexor.isBusy() ? "YES" : "NO");
        } else if (indexorStuckDetectionActive) {
            int currentPos = indexor.getCurrentPosition();
            int posChange = Math.abs(currentPos - lastIndexorPosition);
            telemetry.addData("Stuck Detection", "ACTIVE (%.1fs)", indexorTimer.seconds());
            telemetry.addData("Position Change", "%d ticks", posChange);
        } else if (indexor.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.FLOAT) {
            // Check if it's floating due to reaching position or being stuck
            if (Math.abs(indexor.getPower()) < 0.01 && !indexorStuckDetectionActive) {
                telemetry.addData("Indexor Status", "🎯 FLOATING (Ready for trigger movement)");
            } else {
                telemetry.addData("Indexor Status", "⚠️ FLOATING (Stuck detected)");
            }
        } else {
            telemetry.addData("Stuck Detection", "INACTIVE");
        }
        telemetry.addData("", "");
        
        // Show chassis motor powers
        telemetry.addData("Front Left", "%.2f", leftFront.getPower());
        telemetry.addData("Front Right", "%.2f", rightFront.getPower());
        telemetry.addData("Back Left", "%.2f", leftBack.getPower());
        telemetry.addData("Back Right", "%.2f", rightBack.getPower());
        
        telemetry.update();
    }
}