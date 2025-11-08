package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

// AprilTag imports
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

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
    
    // Declare color sensors for ball detection and intake management
    private NormalizedColorSensor colorSensorIntake;    // Position 1: Entry point detection
    private NormalizedColorSensor colorSensorFire;      // Position 2: Exit/firing point detection  
    private NormalizedColorSensor colorSensorStore;     // Position 3: Middle section detection
    
    // Declare mecanum drive motors
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    
    // Variables to track button states
    private boolean previousA = false;  // Gamepad1 A button (intake)
    private boolean previousX1 = false; // Gamepad1 X button (indexor)
    private boolean previousY1 = false; // Gamepad1 Y button (shooter 1300)
    private boolean previousB1 = false; // Gamepad1 B button (shooter 1530)
    
    // Gamepad2 button states (operator controls)
    private boolean previousX2 = false; // Gamepad2 X button (indexor)
    private boolean previousY2 = false; // Gamepad2 Y button (shooter)
    private boolean previousB2 = false; // Gamepad2 B button (trigger)
    
    // Shooter state tracking
    private boolean shooterIntentionallyRunning = false;
    private double currentShooterTargetVelocity = 1300; // Track current target velocity
    private ElapsedTime shooterStabilizationTimer = new ElapsedTime(); // Timer for speed stabilization
    private boolean shooterSpeedStable = false; // Track if speed is stable
    
    // Fast spin-up tracking
    private ElapsedTime shooterBoostTimer = new ElapsedTime();      // Timer for initial boost phase
    private boolean shooterBoostPhaseActive = false;               // Track if boost phase is active
    
    // Trigger servo management
    private boolean manualTriggerControl = false; // Track if trigger is under manual control
    private ElapsedTime triggerManualTimer = new ElapsedTime(); // Timer for manual control timeout
    private boolean triggerAutoFireSequence = false; // Track if auto fire sequence is active
    private ElapsedTime triggerFireTimer = new ElapsedTime(); // Timer for fire sequence
    
    // Indexor stuck detection variables
    private ElapsedTime indexorTimer = new ElapsedTime();
    private int lastIndexorPosition = 0;
    private boolean indexorStuckDetectionActive = false;
    
    // Conveyor unsticking sequence variables
    private boolean conveyorUnstickingActive = false;      // Track if conveyor unsticking is in progress
    private ElapsedTime conveyorUnstickTimer = new ElapsedTime(); // Timer for unsticking sequence
    private boolean conveyorReversePhase = true;           // Track which phase of unsticking (reverse/forward)
    
    // AprilTag detection and auto-alignment
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private boolean autoAlignmentActive = false; // Track if auto-alignment is active
    private ElapsedTime autoAlignmentTimer = new ElapsedTime(); // Timer for alignment timeout
    private boolean previousA2 = false; // Gamepad2 A button (auto-alignment toggle)
    
    // Indexor position control variables
    private boolean indexorRunningToPosition = false;
    
    // Global indexor position tracking
    private int indexorGlobalPosition = 0;                    // Track position count (MOD-based precise positioning)
    private int indexorInitialPosition = 0;                   // Store initial encoder position for reference
    
    // Manual indexor control variables
    private boolean manualIndexorControl = false;             // Track if indexor is under manual control
    
    // Ball counting and intake management variables
    private int ballCount = 0;                                 // Current number of balls in system (0-3)
    private boolean intakeRunning = false;                     // Track if intake is currently running
    private boolean ballDetectedIntake = false;               // Ball detected at intake sensor
    private boolean ballDetectedFire = false;                 // Ball detected at fire sensor  
    private boolean ballDetectedStore = false;                // Ball detected at store sensor
    private boolean previousBallDetectedIntake = false;       // Previous state for edge detection
    private boolean previousBallDetectedFire = false;         // Previous state for edge detection
    private boolean previousBallDetectedStore = false;        // Previous state for edge detection
    private boolean intakeSafetyStop = false;                 // Emergency stop when 3 balls detected
    private ElapsedTime stuckDetectionTimer = new ElapsedTime(); // Timer for indexor stuck detection
    private int indexorLastPosition = 0;                      // Last recorded indexor position for stuck detection
    
    // Motor power settings
    public static final double INTAKE_POWER = 0.8;
    public static final double CONVEYOR_POWER = 1.0;
    public static final double AUTO_INDEXOR_POWER = 0.4;      // Power for automatic indexor movement
    public static final double MANUAL_INDEXOR_POWER = 0.4;    // Power for manual indexor control via joystick
    public static final double SHOOTER_POWER = 1.0;
    public static final double SHOOTER_SERVO_POWER = 1.0;     // Positive for forward direction
    
    // Indexor position settings - Precise position-based calculation
    public static final double INDEXOR_TICKS_PER_REVOLUTION = 537.7;  // goBILDA 312 RPM motor precise value
    public static final double INDEXOR_DENOMINATOR = INDEXOR_TICKS_PER_REVOLUTION / 3.0;  // 179.23 ticks per 120°
    public static final int INDEXOR_TICKS_PER_120_DEGREES = (int) Math.round(INDEXOR_DENOMINATOR);  // 179 ticks (rounded)
    public static final int INDEXOR_POSITION_0 = 0;               // 0 degrees (home position)
    public static final int INDEXOR_POSITION_1 = 179;             // 120 degrees
    public static final int INDEXOR_POSITION_2 = 358;             // 240 degrees  
    public static final int INDEXOR_POSITION_3 = 537;             // 360 degrees
    public static final int[] INDEXOR_POSITIONS = {INDEXOR_POSITION_0, INDEXOR_POSITION_1, INDEXOR_POSITION_2, INDEXOR_POSITION_3};
    
    // Shooter velocity control (ticks per second)
    public static double SHOOTER_TARGET_VELOCITY = 1300;      // Range: 1200-1800 ticks/sec
    public static final double SHOOTER_SPEED_THRESHOLD = 0.95; // 95% of target speed
    public static final double SHOOTER_TICKS_PER_REVOLUTION = 1020.0; // goBILDA 435 RPM motor
    
    // Speed consistency settings
    public static final double SHOOTER_MIN_SPEED_THRESHOLD = 0.85; // 85% minimum for consistent shooting
    public static final double SHOOTER_STABILIZATION_TIME = 1.0;   // Seconds to wait for speed stabilization
    public static final double SHOOTER_SPEED_TOLERANCE = 50;       // ticks/sec tolerance for "stable" speed
    
    // Advanced shooter tuning (FTC Dashboard configurable)
    public static double SHOOTER_B_BUTTON_VELOCITY = 1530;         // B button target velocity (configurable)
    public static double SHOOTER_VELOCITY_CORRECTION_FACTOR = 1.02; // Slight overcorrection for consistency
    
    // Fast spin-up optimization settings
    public static double SHOOTER_INITIAL_BOOST_MULTIPLIER = 1.15;   // 15% initial boost for faster ramp-up
    public static double SHOOTER_BOOST_DURATION = 0.5;             // Seconds to apply initial boost
    public static double SHOOTER_AGGRESSIVE_RAMP_MULTIPLIER = 1.08; // 8% aggressive ramp for first few seconds
    
    // Speed light control settings (using servo positions for LED control)
    public static final double LIGHT_OFF_POSITION = 0.0;      // Servo position for light off
    public static final double LIGHT_GREEN_POSITION = 0.5;    // Servo position for green light
    public static final double LIGHT_WHITE_POSITION = 1.0;    // Servo position for white light
    
    // Trigger servo positions
    public static final double TRIGGER_FIRE = 0.15;     // 27 degrees (27/180 = 0.15) - FIRE POSITION (compact)
    public static final double TRIGGER_HOME = 0.76;     // 137 degrees (137/180 = 0.76) - HOME/SAFE POSITION (retracted)
    
    // Trigger management
    public static final double MANUAL_TRIGGER_TIMEOUT = 3.0;  // Seconds before auto-management resumes
    public static final double TRIGGER_FIRE_DURATION = 0.5;   // Seconds to stay in fire position before returning home
    
    // Stuck detection settings
    public static final double INDEXOR_STUCK_TIMEOUT = 3.0;    // seconds - time before considering indexor stuck
    public static final int INDEXOR_PROGRESS_THRESHOLD = 20;   // minimum ticks of progress required
    
    // Color sensor and ball detection settings
    public static final double COLOR_SENSOR_GAIN = 15.0;       // Sensor gain for better detection
    public static final double BALL_DETECTION_THRESHOLD = 0.15; // Alpha threshold for ball detection (0.0-1.0)
    public static final int MAX_BALLS_ALLOWED = 3;             // Maximum balls in system
    public static final double INTAKE_STUCK_DETECTION_TIMEOUT = 2.0;  // Seconds to detect indexor stuck during intake
    public static final int INTAKE_STUCK_PROGRESS_THRESHOLD = 10;     // Minimum encoder progress to avoid stuck detection
    public static final double INTAKE_CONVEYOR_REVERSE_DURATION = 1.0; // Seconds to reverse conveyor when stuck
    
    // LED control settings for ball count feedback  
    public static final double LIGHT_RED_POSITION = 0.25;      // Servo position for red light (3 balls detected)
    
    // Conveyor unsticking settings
    public static final double CONVEYOR_REVERSE_DURATION = 0.3;   // seconds to reverse conveyor when unsticking
    public static final double CONVEYOR_FORWARD_DURATION = 0.2;   // seconds to run forward after reverse
    public static final double CONVEYOR_UNSTICK_POWER = -0.8;     // negative for reverse direction
    
    // Mecanum drive settings
    public static final double DRIVE_SPEED_MULTIPLIER = 0.8;  // Max drive speed (0.0 to 1.0)
    public static final double STRAFE_SPEED_MULTIPLIER = 0.8; // Max strafe speed (0.0 to 1.0)
    public static final double TURN_SPEED_MULTIPLIER = 0.6;   // Max turn speed (0.0 to 1.0)
    
    // Manual control settings
    public static final double JOYSTICK_DEADZONE = 0.1;       // Deadzone for joystick input
    
    // AprilTag auto-alignment settings
    public static final int TARGET_TAG_ID = 20; // Blue AprilTag ID
    public static final double AUTO_ALIGNMENT_TIMEOUT = 5.0;  // Seconds before auto-alignment stops
    public static final double BEARING_ALIGNMENT_TOLERANCE = 2.0; // degrees - how precise bearing alignment needs to be
    public static final double BEARING_ALIGNMENT_POWER = 0.02; // proportional control gain for bearing correction
    public static final double ALIGNMENT_TURN_POWER = 0.3;    // maximum turn power for alignment
    public static final double ALIGNMENT_DRIVE_POWER = 0.25;  // maximum drive power for alignment
    public static final double OPTIMAL_SHOOTING_DISTANCE = 24.0; // inches - optimal distance from AprilTag
    public static final double DISTANCE_TOLERANCE = 6.0;      // inches - acceptable distance variation
    
    @Override
    public void runOpMode() {
        // Initialize motors
        initializeMotors();
        
        // Initialize AprilTag system
        initializeAprilTag();
        
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "DECODE Simple + Intake - Initialized");
        telemetry.addData("=== GAMEPAD 1 (DRIVER) ===", "");
        telemetry.addData("A Button", "Intake + Conveyor + Indexor");
        telemetry.addData("Left Stick", "Drive/Strafe");
        telemetry.addData("Right Stick X", "Turn");
        telemetry.addData("=== GAMEPAD 2 (OPERATOR) ===", "");
        telemetry.addData("A Button", "🎯 Toggle AprilTag Auto-Alignment");
        telemetry.addData("X Button", "Indexor MOD-based Advance (179.23°)");
        telemetry.addData("Y Button", "Shooter + Servo (Auto-stops Intake)");
        telemetry.addData("B Button", "Auto Fire (Fire → Home)");
        telemetry.addData("", "");
        telemetry.addData("🎯 INDEXOR GLOBAL", "Current Position: %d (%.0f°)", indexorGlobalPosition, indexorGlobalPosition * 120.0);
        telemetry.addData("Drive Speed", "%.0f%% max", DRIVE_SPEED_MULTIPLIER * 100);
        telemetry.addData("Strafe Speed", "%.0f%% max", STRAFE_SPEED_MULTIPLIER * 100);
        telemetry.addData("Turn Speed", "%.0f%% max", TURN_SPEED_MULTIPLIER * 100);
        telemetry.addData("Note", "Driver/Operator Split Controls");
        telemetry.update();
        
        waitForStart();
        
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            readColorSensors();     // Read color sensors and manage ball detection
            handleControllerInputs();
            handleManualIndexorControl();
            handleAutoAlignment(); // Handle AprilTag auto-alignment
            handleMecanumDrive();
            checkIndexorStuck();
            handleConveyorUnsticking();
            checkTriggerTimeout();
            handleTriggerAutoFire();
            updateSpeedLight();
            monitorShooterSpeed(); // Monitor and maintain shooter speed
            updateTelemetry();
            sleep(20); // Small delay to prevent excessive CPU usage
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
        
        // Initialize color sensors for ball detection
        colorSensorIntake = hardwareMap.get(NormalizedColorSensor.class, "colorSensorEntry");
        colorSensorFire = hardwareMap.get(NormalizedColorSensor.class, "colorSensorExit");
        colorSensorStore = hardwareMap.get(NormalizedColorSensor.class, "colorSensorMiddle");
        
        // Set color sensor gain for better detection
        colorSensorIntake.setGain((float)COLOR_SENSOR_GAIN);
        colorSensorFire.setGain((float)COLOR_SENSOR_GAIN);
        colorSensorStore.setGain((float)COLOR_SENSOR_GAIN);
        
        // Enable LED lights on color sensors if available
        if (colorSensorIntake instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorIntake).enableLight(true);
        }
        if (colorSensorFire instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorFire).enableLight(true);
        }
        if (colorSensorStore instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorStore).enableLight(true);
        }
        
        // Set mecanum drive motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        
        // Set zero power behavior
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        // Set mecanum drive motor zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Reset encoders and initialize global position
        indexor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Initialize indexor global position tracking
        indexorInitialPosition = indexor.getCurrentPosition(); // Should be 0 after reset
        indexorGlobalPosition = 0; // Start at position 0 (home position)
        
        // Set shooter mode with optimized configuration for fast spin-up
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Configure shooter motor for optimal velocity control
        // These settings help the motor reach target velocity faster
        shooter.setVelocityPIDFCoefficients(32, 0, 3, 12);  // Aggressive PID for fast ramp-up
        shooter.setPositionPIDFCoefficients(5.0);            // Standard position PID
        
        // Set mecanum drive motors to run without encoders for teleop
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set indexor to use encoder
        indexor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    private void handleControllerInputs() {
        // Get current button states for gamepad1 (driver)
        boolean currentA = gamepad1.a;
        boolean currentX1 = gamepad1.x;
        boolean currentY1 = gamepad1.y;
        boolean currentB1 = gamepad1.b;
        
        // Get current button states for gamepad2 (operator)
        boolean currentA2 = gamepad2.a;
        boolean currentX2 = gamepad2.x;
        boolean currentY2 = gamepad2.y;
        boolean currentB2 = gamepad2.b;
        
        // Handle A button on gamepad1 - Toggle Intake, Conveyor, and Indexor
        if (currentA && !previousA) {
            toggleIntakeSystem();
        }
        
        // Handle X button on gamepad1 - Start Indexor
        if (currentX1 && !previousX1) {
            startIndexor();
        }
        
        // Handle Y button on gamepad1 - Toggle Shooter at 1300 velocity
        if (currentY1 && !previousY1) {
            toggleShooter1300();
        }
        
        // Handle B button on gamepad1 - Toggle Shooter at 1530 velocity
        if (currentB1 && !previousB1) {
            toggleShooter1530();
        }
        
        // Handle X button on gamepad2 - Run Indexor to Position
        if (currentX2 && !previousX2) {
            runIndexorToPosition();
        }
        
        // Handle A button on gamepad2 - Toggle AprilTag Auto-Alignment
        if (currentA2 && !previousA2) {
            telemetry.addData("🎮 DEBUG", "Gamepad2 A button pressed!");
            telemetry.update();
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
        
        // Update previous button states for gamepad1
        previousA = currentA;
        previousX1 = currentX1;
        previousY1 = currentY1;
        previousB1 = currentB1;
        
        // Update previous button states for gamepad2
        previousA2 = currentA2;
        previousX2 = currentX2;
        previousY2 = currentY2;
        previousB2 = currentB2;
    }
    
    private void toggleIntakeSystem() {
        // Use the enhanced ball count management version
        toggleIntakeSystemWithBallCount();
    }
    
    private void startIndexor() {
        // Check if indexor is already running
        boolean indexorRunning = Math.abs(indexor.getPower()) > 0.1;
        
        if (indexorRunning) {
            // Stop indexor
            indexor.setPower(0);
            indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            indexor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            indexorStuckDetectionActive = false;
            indexorRunningToPosition = false;
            
            // Cancel any active unsticking sequence
            conveyorUnstickingActive = false;
            
            telemetry.addData("Indexor", "STOPPED");
        } else {
            // Start indexor with automatic power
            indexor.setPower(AUTO_INDEXOR_POWER);
            indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            startIndexorStuckDetection();
            
            // Cancel any active unsticking sequence
            conveyorUnstickingActive = false;
            
            telemetry.addData("Indexor", "STARTED");
            telemetry.addData("Indexor Power", "%.1f", AUTO_INDEXOR_POWER);
        }
        telemetry.update();
    }
    
    private void runIndexorToPosition() {
        // Check if manual control is active - don't allow automatic positioning
        if (manualIndexorControl) {
            telemetry.addData("🎮 X Button Blocked", "Manual joystick control is active");
            telemetry.addData("💡 Tip", "Release left joystick to use X button");
            telemetry.update();
            return;
        }
        
        // Stop any continuous power mode first
        indexor.setPower(0);
        
        // Cancel any active unsticking sequence
        conveyorUnstickingActive = false;
        
        // Start conveyor to help feed balls through the system
        conveyor.setPower(CONVEYOR_POWER);
        
        // Get current encoder position and calculate next position using MOD logic
        int currentPosition = indexor.getCurrentPosition();
        double remainder = currentPosition % INDEXOR_DENOMINATOR;
        int nextIncrement = (int) Math.round(INDEXOR_DENOMINATOR - remainder);
        if (nextIncrement == 0) {
            nextIncrement = (int) Math.round(INDEXOR_DENOMINATOR); // If exactly on position, move to next
        }
        
        // If remainder is above 150, add one more denominator (skip to next-next position)
        if (remainder > 150) {
            nextIncrement += (int) Math.round(INDEXOR_DENOMINATOR);
        }
        
        int targetPosition = currentPosition + nextIncrement;
        
        // Update global position for tracking (increment for telemetry)
        indexorGlobalPosition = indexorGlobalPosition + 1;
        
        // Reset motor behavior to FLOAT and set to position mode
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        indexor.setTargetPosition(targetPosition);
        indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexor.setPower(AUTO_INDEXOR_POWER);
        
        // Start tracking
        indexorRunningToPosition = true;
        startIndexorStuckDetection();
        
        telemetry.addData("🎯 INDEXOR MOD CALC", "Current: %d, Remainder: %.1f, Next: +%d%s", 
            currentPosition, remainder, nextIncrement, remainder > 150 ? " (Skip)" : "");
        telemetry.addData("Target Position", "%d ticks (Global: %d)", targetPosition, indexorGlobalPosition);
        telemetry.addData("Movement", "MOD-based precise positioning (FLOAT)");
        telemetry.addData("Conveyor", "RUNNING at %.1f power", CONVEYOR_POWER);
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
            
            // Stop conveyor when indexor positioning is complete
            conveyor.setPower(0);
            
            telemetry.addData("✅ INDEXOR", "Position reached");
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
                // Indexor is stuck - start conveyor unsticking sequence
                indexor.setPower(0);
                indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                indexorStuckDetectionActive = false;
                indexorRunningToPosition = false;
                
                // Start conveyor unsticking sequence
                conveyorUnstickingActive = true;
                conveyorReversePhase = true; // Start with reverse
                conveyorUnstickTimer.reset();
                conveyor.setPower(CONVEYOR_UNSTICK_POWER); // Reverse conveyor
                
                telemetry.addData("⚠️ INDEXOR STUCK", "Starting conveyor unstick sequence");
                telemetry.addData("🔄 CONVEYOR", "REVERSING to unstick (%.1fs)", CONVEYOR_REVERSE_DURATION);
                telemetry.addData("Position Change", "%d ticks in %.1fs", positionChange, timeElapsed);
            } else {
                // Reset detection timer if making progress
                indexorTimer.reset();
                lastIndexorPosition = currentPosition;
            }
        }
    }
    
    private void handleConveyorUnsticking() {
        if (!conveyorUnstickingActive) {
            return; // No unsticking sequence active
        }
        
        double elapsedTime = conveyorUnstickTimer.seconds();
        
        if (conveyorReversePhase) {
            // Phase 1: Reverse conveyor to unstick
            if (elapsedTime >= CONVEYOR_REVERSE_DURATION) {
                // Switch to forward phase
                conveyorReversePhase = false;
                conveyorUnstickTimer.reset(); // Reset timer for forward phase
                conveyor.setPower(CONVEYOR_POWER); // Forward direction
                
                telemetry.addData("🔄 CONVEYOR", "FORWARD phase (%.1fs)", CONVEYOR_FORWARD_DURATION);
            }
        } else {
            // Phase 2: Forward conveyor briefly
            if (elapsedTime >= CONVEYOR_FORWARD_DURATION) {
                // End unsticking sequence
                conveyorUnstickingActive = false;
                conveyor.setPower(0); // Stop conveyor
                
                telemetry.addData("✅ UNSTICK COMPLETE", "Conveyor sequence finished");
                telemetry.addData("💡 Action", "Motor floating - press X or A to restart");
            }
        }
    }
    
    private void startShooterWithFastSpinUp(double targetVelocity) {
        // STEP 1: Stop intake system first for safety and performance
        boolean intakeWasRunning = Math.abs(intake.getPower()) > 0.1;
        if (intakeWasRunning) {
            intake.setPower(0);
            conveyor.setPower(0);
            indexor.setPower(0);
            telemetry.addData("🛑 INTAKE", "Auto-stopped for shooter startup");
        }
        
        // STEP 2: Start shooter with fast spin-up technique
        shooterIntentionallyRunning = true;
        currentShooterTargetVelocity = targetVelocity;
        
        // Initialize fast spin-up sequence
        shooterBoostPhaseActive = true;
        shooterBoostTimer.reset();
        shooterStabilizationTimer.reset();
        shooterSpeedStable = false;
        
        // Apply initial boost for faster ramp-up (15% above target initially)
        double boostVelocity = targetVelocity * SHOOTER_INITIAL_BOOST_MULTIPLIER;
        shooter.setVelocity(boostVelocity);
        shooterServo.setPower(SHOOTER_SERVO_POWER);
        
        telemetry.addData("🚀 FAST STARTUP", "Boosting to %.0f ticks/sec", boostVelocity);
        telemetry.addData("🎯 Target", "%.0f ticks/sec", targetVelocity);
        if (intakeWasRunning) {
            telemetry.addData("💡 Note", "Intake was stopped automatically");
        }
    }
    
    private void toggleShooter() {
        // Check if shooter is currently running
        if (shooterIntentionallyRunning) {
            // Stop shooter and servo
            shooter.setPower(0);
            shooterServo.setPower(0);
            shooterIntentionallyRunning = false;
            shooterBoostPhaseActive = false; // Reset boost phase
            
            telemetry.addData("Shooter System", "STOPPED");
        } else {
            // Start shooter with fast spin-up
            startShooterWithFastSpinUp(SHOOTER_TARGET_VELOCITY);
            telemetry.addData("Shooter System", "FAST STARTUP at %.0f ticks/sec", SHOOTER_TARGET_VELOCITY);
        }
        telemetry.update();
    }
    
    private void toggleShooter1530() {
        // Check if shooter is currently running
        if (shooterIntentionallyRunning) {
            // Stop shooter and servo
            shooter.setPower(0);
            shooterServo.setPower(0);
            shooterIntentionallyRunning = false;
            shooterSpeedStable = false; // Reset stability status
            shooterBoostPhaseActive = false; // Reset boost phase
            
            telemetry.addData("Shooter System", "STOPPED");
        } else {
            // Start shooter with fast spin-up at high velocity
            startShooterWithFastSpinUp(SHOOTER_B_BUTTON_VELOCITY);
            telemetry.addData("Shooter System", "FAST STARTUP at %.0f ticks/sec", SHOOTER_B_BUTTON_VELOCITY);
            telemetry.addData("Status", "� Fast ramp-up active...");
        }
        telemetry.update();
    }
    
    private void toggleShooter1300() {
        // Check if shooter is currently running
        if (shooterIntentionallyRunning) {
            // Stop shooter and servo
            shooter.setPower(0);
            shooterServo.setPower(0);
            shooterIntentionallyRunning = false;
            shooterSpeedStable = false; // Reset stability status
            shooterBoostPhaseActive = false; // Reset boost phase
            
            telemetry.addData("Shooter System", "STOPPED");
        } else {
            // Start shooter with fast spin-up at 1300 ticks/sec
            startShooterWithFastSpinUp(1300);
            telemetry.addData("Shooter System", "FAST STARTUP at 1300 ticks/sec");
            telemetry.addData("Status", "� Fast ramp-up active...");
        }
        telemetry.update();
    }
    
    private void monitorShooterSpeed() {
        if (!shooterIntentionallyRunning) {
            shooterSpeedStable = false;
            shooterBoostPhaseActive = false;
            return;
        }
        
        double currentVelocity = shooter.getVelocity();
        double speedError = Math.abs(currentVelocity - currentShooterTargetVelocity);
        double stabilizationTime = shooterStabilizationTimer.seconds();
        double boostTime = shooterBoostTimer.seconds();
        
        // Handle boost phase for fast startup
        if (shooterBoostPhaseActive) {
            if (boostTime >= SHOOTER_BOOST_DURATION) {
                // End boost phase, transition to target velocity
                shooterBoostPhaseActive = false;
                shooter.setVelocity(currentShooterTargetVelocity);
                telemetry.addData("🎯 BOOST COMPLETE", "Transitioning to target velocity");
            } else if (boostTime > 0.1) { // Small delay to avoid immediate check
                // Check if we're getting close to target during boost
                double targetSpeedPercentage = currentVelocity / currentShooterTargetVelocity;
                if (targetSpeedPercentage >= 0.9) {
                    // We're at 90%+ of target speed, end boost early
                    shooterBoostPhaseActive = false;
                    shooter.setVelocity(currentShooterTargetVelocity);
                    telemetry.addData("🎯 EARLY BOOST END", "Target reached fast!");
                }
            }
            return; // Skip normal monitoring during boost phase
        }
        
        // Normal speed monitoring (post-boost)
        boolean speedWithinTolerance = speedError <= SHOOTER_SPEED_TOLERANCE;
        
        if (speedWithinTolerance && stabilizationTime >= SHOOTER_STABILIZATION_TIME) {
            shooterSpeedStable = true;
        } else if (!speedWithinTolerance) {
            // Speed drifted - reset stabilization timer
            shooterStabilizationTimer.reset();
            shooterSpeedStable = false;
            
            // Apply corrective velocity command with slight overcorrection for consistency
            double correctedVelocity = currentShooterTargetVelocity * SHOOTER_VELOCITY_CORRECTION_FACTOR;
            shooter.setVelocity(correctedVelocity);
        }
    }
    
    private void updateSpeedLight() {
        // Priority 1: If 3 balls detected, show RED light regardless of shooter status
        if (intakeSafetyStop || ballCount >= MAX_BALLS_ALLOWED) {
            speedLight.setPosition(LIGHT_RED_POSITION);
            return;
        }
        
        // Priority 2: Standard shooter speed indication
        if (!shooterIntentionallyRunning) {
            // Shooter is off - speed light should be off
            speedLight.setPosition(LIGHT_OFF_POSITION);
            return;
        }
        
        // Shooter is on - check speed and stability
        double currentVelocity = shooter.getVelocity();
        double speedPercentage = currentVelocity / currentShooterTargetVelocity;
        
        if (shooterSpeedStable && speedPercentage > SHOOTER_SPEED_THRESHOLD) {
            // Speed is optimal and stable - green light (95%+ and stable)
            speedLight.setPosition(LIGHT_GREEN_POSITION);
        } else if (currentVelocity > 50 && speedPercentage > SHOOTER_MIN_SPEED_THRESHOLD) {
            // Speed is acceptable but may not be stable - white light (85%+ of target)
            speedLight.setPosition(LIGHT_WHITE_POSITION);
        } else {
            // Speed is too low for consistent shooting - off (no light)
            speedLight.setPosition(LIGHT_OFF_POSITION);
        }
    }
    
    private void toggleTriggerServo() {
        // Check if intake is running - safety first!
        boolean intakeRunning = Math.abs(intake.getPower()) > 0.1;
        if (intakeRunning) {
            telemetry.addData("🚫 TRIGGER BLOCKED", "Cannot move trigger while intake is running");
            telemetry.addData("💡 Safety Tip", "Stop intake (Gamepad1 A button) before using trigger");
            telemetry.update();
            return;
        }
        
        // Start auto fire sequence
        triggerAutoFireSequence = true;
        triggerFireTimer.reset();
        
        // Move to FIRE position
        triggerServo.setPosition(TRIGGER_FIRE);
        telemetry.addData("🎯 AUTO FIRE", "Moving to FIRE position (27°)");
        telemetry.addData("⏱️ Timer", "Will return to HOME in %.1fs", TRIGGER_FIRE_DURATION);
        telemetry.update();
    }
    
    private void checkTriggerTimeout() {
        // Check if manual trigger control has timed out
        if (manualTriggerControl && triggerManualTimer.seconds() > MANUAL_TRIGGER_TIMEOUT) {
            manualTriggerControl = false;
            
            // Get current trigger position
            double currentTriggerPosition = triggerServo.getPosition();
            boolean triggerInFirePosition = Math.abs(currentTriggerPosition - TRIGGER_FIRE) < 0.05;
            
            // If trigger is in fire position and intake is not running, move to home for safety
            boolean intakeRunning = Math.abs(intake.getPower()) > 0.1;
            if (triggerInFirePosition && !intakeRunning) {
                // Check if it's safe to move trigger to home position
                if (Math.abs(currentTriggerPosition - TRIGGER_HOME) > 0.05) {
                    triggerServo.setPosition(TRIGGER_HOME);
                }
            }
        }
    }
    
    private void handleTriggerAutoFire() {
        // Handle automatic fire sequence
        if (triggerAutoFireSequence && triggerFireTimer.seconds() >= TRIGGER_FIRE_DURATION) {
            // Time to return to home position
            triggerServo.setPosition(TRIGGER_HOME);
            triggerAutoFireSequence = false;
            
            // Move indexor to next position after trigger sequence completes
            moveIndexorAfterTrigger();
            
            telemetry.addData("🏠 AUTO RETURN", "Trigger returned to HOME position (137°)");
            telemetry.addData("✅ COMPLETE", "Fire sequence finished");
            telemetry.addData("🔄 INDEXOR", "Moving to next position...");
            telemetry.update();
        }
    }
    
    private void moveIndexorAfterTrigger() {
        // Check if manual control is active - don't allow automatic positioning
        if (manualIndexorControl) {
            telemetry.addData("🎮 Indexor Blocked", "Manual joystick control is active");
            return;
        }
        
        // Stop any continuous power mode first
        indexor.setPower(0);
        
        // Cancel any active unsticking sequence
        conveyorUnstickingActive = false;
        
        // Start conveyor to help feed balls through the system
        conveyor.setPower(CONVEYOR_POWER);
        
        // Get current encoder position and calculate next position using MOD logic
        int currentPosition = indexor.getCurrentPosition();
        double remainder = currentPosition % INDEXOR_DENOMINATOR;
        int nextIncrement = (int) Math.round(INDEXOR_DENOMINATOR - remainder);
        if (nextIncrement == 0) {
            nextIncrement = (int) Math.round(INDEXOR_DENOMINATOR); // If exactly on position, move to next
        }
        
        // If remainder is above 150, add one more denominator (skip to next-next position)
        if (remainder > 150) {
            nextIncrement += (int) Math.round(INDEXOR_DENOMINATOR);
        }
        
        int targetPosition = currentPosition + nextIncrement;
        
        // Update global position for tracking (increment for telemetry)
        indexorGlobalPosition = indexorGlobalPosition + 1;
        
        // Reset motor behavior to FLOAT and set to position mode
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        indexor.setTargetPosition(targetPosition);
        indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexor.setPower(AUTO_INDEXOR_POWER);
        
        // Start tracking
        indexorRunningToPosition = true;
        startIndexorStuckDetection();
        
        telemetry.addData("🔄 TRIGGER INDEXOR", "Current: %d, Target: %d (+%d)%s", 
            currentPosition, targetPosition, nextIncrement, remainder > 150 ? " Skip" : "");
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
        
        // Show ball detection and intake management status
        telemetry.addData("🎾 Ball Count", "%d/%d", ballCount, MAX_BALLS_ALLOWED);
        
        // Show color sensor detection status
        String intakeStatus = ballDetectedIntake ? "🔴 DETECTED" : "⚪ CLEAR";
        String fireStatus = ballDetectedFire ? "🔴 DETECTED" : "⚪ CLEAR";  
        String storeStatus = ballDetectedStore ? "🔴 DETECTED" : "⚪ CLEAR";
        telemetry.addData("Sensors", "Entry: %s | Fire: %s | Store: %s", intakeStatus, fireStatus, storeStatus);
        
        // Show intake safety status
        if (intakeSafetyStop) {
            telemetry.addData("🛑 INTAKE STATUS", "SAFETY STOP - System FULL!");
            telemetry.addData("💡 Action", "Fire balls to resume intake");
        } else if (intakeRunning) {
            telemetry.addData("✅ INTAKE STATUS", "RUNNING - Ball count: %d", ballCount);
        } else {
            telemetry.addData("⏹️ INTAKE STATUS", "STOPPED");
        }
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
            telemetry.addData("Target Velocity", "%.0f ticks/sec", currentShooterTargetVelocity);
            telemetry.addData("Current Velocity", "%.0f ticks/sec", shooter.getVelocity());
            double speedPercentage = shooter.getVelocity() / currentShooterTargetVelocity * 100;
            telemetry.addData("Speed Accuracy", "%.1f%%", speedPercentage);
            
            // Speed stabilization status
            double stabilizationTime = shooterStabilizationTimer.seconds();
            double speedError = Math.abs(shooter.getVelocity() - currentShooterTargetVelocity);
            double boostTime = shooterBoostTimer.seconds();
            
            if (shooterBoostPhaseActive) {
                double boostTimeLeft = SHOOTER_BOOST_DURATION - boostTime;
                double currentBoostVelocity = currentShooterTargetVelocity * SHOOTER_INITIAL_BOOST_MULTIPLIER;
                telemetry.addData("🚀 FAST STARTUP", "Boost phase (%.1fs left)", boostTimeLeft);
                telemetry.addData("🔥 Boost Velocity", "%.0f ticks/sec (%.0f%% above target)", 
                    currentBoostVelocity, (SHOOTER_INITIAL_BOOST_MULTIPLIER - 1) * 100);
                telemetry.addData("⚡ Speed Progress", "%.1f%% of target reached", 
                    (shooter.getVelocity() / currentShooterTargetVelocity) * 100);
            } else if (shooterSpeedStable) {
                telemetry.addData("🎯 STABILITY", "STABLE - Ready to shoot!");
                telemetry.addData("✅ SPEED STATUS", "Optimal & consistent");
            } else if (stabilizationTime < SHOOTER_STABILIZATION_TIME) {
                double timeLeft = SHOOTER_STABILIZATION_TIME - stabilizationTime;
                telemetry.addData("🟡 STABILITY", "Stabilizing... %.1fs left", timeLeft);
            } else {
                telemetry.addData("⚠️ STABILITY", "Speed varying (±%.0f ticks/sec)", speedError);
            }
            
            // Overall speed consistency warning
            if (speedPercentage < SHOOTER_MIN_SPEED_THRESHOLD * 100) {
                telemetry.addData("⚠️ SPEED WARNING", "Too slow for consistent shots!");
            } else if (shooterSpeedStable && speedPercentage >= SHOOTER_SPEED_THRESHOLD * 100) {
                telemetry.addData("✅ READY TO SHOOT", "Speed stable & optimal");
            }
            
            telemetry.addData("Shooter Servo", "%.2f", shooterServo.getPower());
            telemetry.addData("Speed Light", "%.2f", speedLight.getPosition());
        } else {
            telemetry.addData("Shooter", "STOPPED");
        }
        telemetry.addData("", "");
        
        // Show trigger status
        double currentTriggerPosition = triggerServo.getPosition();
        boolean triggerInFirePosition = Math.abs(currentTriggerPosition - TRIGGER_FIRE) < 0.05;
        if (triggerInFirePosition) {
            telemetry.addData("Trigger Position", "🎯 FIRE (%.2f)", currentTriggerPosition);
        } else {
            telemetry.addData("Trigger Position", "🏠 HOME (%.2f)", currentTriggerPosition);
        }
        
        if (triggerAutoFireSequence) {
            double timeLeft = TRIGGER_FIRE_DURATION - triggerFireTimer.seconds();
            if (timeLeft > 0) {
                telemetry.addData("🎯 Auto Fire", "Returning to HOME in %.1fs", timeLeft);
            } else {
                telemetry.addData("🎯 Auto Fire", "Completing sequence...");
            }
        } else if (manualTriggerControl) {
            double timeLeft = MANUAL_TRIGGER_TIMEOUT - triggerManualTimer.seconds();
            if (timeLeft > 0) {
                telemetry.addData("Manual Trigger", "%.1fs remaining", timeLeft);
            } else {
                telemetry.addData("Manual Trigger", "TIMEOUT - Auto mode resumed");
            }
        }
        
        // Show indexor global position and status
        telemetry.addData("🎯 INDEXOR GLOBAL", "Position %d (%.0f°)", indexorGlobalPosition, indexorGlobalPosition * 120.0);
        telemetry.addData("Next Position", "Position %d (%.0f°)", (indexorGlobalPosition + 1) % 4, ((indexorGlobalPosition + 1) % 4) * 120.0);
        
        // Show stuck detection status
        if (conveyorUnstickingActive) {
            // Show unsticking sequence status
            double timeElapsed = conveyorUnstickTimer.seconds();
            if (conveyorReversePhase) {
                double timeLeft = CONVEYOR_REVERSE_DURATION - timeElapsed;
                telemetry.addData("🔄 UNSTICKING", "REVERSING conveyor (%.1fs left)", timeLeft);
                telemetry.addData("Conveyor Power", "%.2f (REVERSE)", CONVEYOR_UNSTICK_POWER);
            } else {
                double timeLeft = CONVEYOR_FORWARD_DURATION - timeElapsed;
                telemetry.addData("🔄 UNSTICKING", "FORWARD conveyor (%.1fs left)", timeLeft);
                telemetry.addData("Conveyor Power", "%.2f (FORWARD)", CONVEYOR_POWER);
            }
            telemetry.addData("Indexor Status", "FLOATING (stuck detected)");
        } else if (indexorRunningToPosition) {
            telemetry.addData("Indexor Mode", "GLOBAL POSITION CONTROL");
            telemetry.addData("Current Position", "%d ticks", indexor.getCurrentPosition());
            telemetry.addData("Target Position", "%d ticks", indexor.getTargetPosition());
            telemetry.addData("Is Busy", "%s", indexor.isBusy() ? "YES" : "NO");
        } else if (indexorStuckDetectionActive) {
            int currentPos = indexor.getCurrentPosition();
            int posChange = Math.abs(currentPos - lastIndexorPosition);
            telemetry.addData("Stuck Detection", "ACTIVE (%.1fs)", indexorTimer.seconds());
            telemetry.addData("Position Change", "%d ticks", posChange);
        } else if (indexor.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.FLOAT) {
            telemetry.addData("Indexor Status", "FLOATING (stuck detected)");
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
    
    private void handleManualIndexorControl() {
        // Get gamepad2 left joystick Y value for manual indexor control
        double joystickY = -gamepad2.left_stick_y; // Negative for intuitive control (up = forward)
        
        // Apply deadzone
        if (Math.abs(joystickY) < JOYSTICK_DEADZONE) {
            joystickY = 0.0;
        }
        
        // Check if joystick is being actively used
        boolean joystickActive = Math.abs(joystickY) > 0.0;
        
        if (joystickActive) {
            // Joystick is active - switch to manual control
            if (!manualIndexorControl) {
                // Stop any automated indexor movement
                if (indexorRunningToPosition) {
                    indexor.setPower(0);
                    indexor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    indexorRunningToPosition = false;
                    telemetry.addData("Indexor", "Manual control override");
                }
                
                // Cancel any active unsticking sequence
                conveyorUnstickingActive = false;
                
                manualIndexorControl = true;
            }
            
            // Set manual indexor power (slow movement)
            double indexorPower = Math.abs(joystickY) * MANUAL_INDEXOR_POWER;
            indexor.setPower(indexorPower);
            
            // Start conveyor matching joystick direction
            conveyor.setPower(joystickY/Math.abs(joystickY)*CONVEYOR_POWER);
            
            // Start intake in same direction as conveyor (reversing from previous synchronized behavior)
            // Conveyor is REVERSE direction, Intake is FORWARD direction, but using same power direction
            intake.setPower(joystickY/Math.abs(joystickY)*INTAKE_POWER);
            
            // Add telemetry for manual control
            telemetry.addData("🎮 Manual Indexor", "Active - Left Joystick");
            telemetry.addData("Joystick Y", "%.3f", joystickY);
            telemetry.addData("Indexor Power", "%.2f (%.0f%%)", indexorPower, indexorPower * 100);
            double conveyorDirection = joystickY/Math.abs(joystickY)*CONVEYOR_POWER;
            double intakeDirection = joystickY/Math.abs(joystickY)*INTAKE_POWER; // Now using same power direction
            telemetry.addData("Conveyor", "%s at %.2f power", 
                conveyorDirection > 0 ? "FORWARD" : "REVERSE", Math.abs(conveyorDirection));
            telemetry.addData("Intake", "%s at %.2f power (reversed direction)", 
                intakeDirection > 0 ? "FORWARD" : "REVERSE", Math.abs(intakeDirection));
            telemetry.addData("Control Mode", "MANUAL - Intake Direction Reversed");
            
        } else if (manualIndexorControl) {
            // Joystick released - stop indexor, conveyor, intake and return to automatic mode
            indexor.setPower(0);
            conveyor.setPower(0);
            intake.setPower(0);
            manualIndexorControl = false;
            telemetry.addData("🎮 Manual Indexor", "Released - Auto Mode");
            telemetry.addData("Conveyor", "STOPPED - Manual Released");
            telemetry.addData("Intake", "STOPPED - Manual Released");
        }
    }
    
    private void initializeAprilTag() {
        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(1156.544, 1156.544, 640.0, 360.0) // fx, fy, cx, cy for 1280x720
                .build();

        try {
            // Create the vision portal
            VisionPortal.Builder builder = new VisionPortal.Builder();
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            builder.setCameraResolution(new android.util.Size(1280, 720));
            builder.enableLiveView(true);
            builder.setAutoStopLiveView(false);
            builder.addProcessor(aprilTag);
            
            // Build the Vision Portal
            visionPortal = builder.build();
            
            telemetry.addData("✅ AprilTag", "Initialized - Looking for Tag %d", TARGET_TAG_ID);
            telemetry.addData("🎥 Camera", "Webcam 1 connected successfully");
        } catch (Exception e) {
            telemetry.addData("❌ AprilTag Error", "Failed to initialize: %s", e.getMessage());
            telemetry.addData("💡 Camera Check", "Ensure 'Webcam 1' is connected and configured");
            visionPortal = null;
            aprilTag = null;
        }
        telemetry.update();
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
            // Check if AprilTag system is ready
            if (visionPortal == null || aprilTag == null) {
                telemetry.addData("❌ Auto-Alignment", "Camera system not ready!");
                telemetry.addData("💡 Check", "Camera connection and configuration");
                telemetry.update();
                return;
            }
            
            // Start auto-alignment
            autoAlignmentActive = true;
            autoAlignmentTimer.reset();
            telemetry.addData("🎯 Auto-Alignment", "STARTED - Looking for AprilTag %d", TARGET_TAG_ID);
            telemetry.addData("💡 Tip", "Point camera toward target tag");
        }
        telemetry.update();
    }
    
    private void handleAutoAlignment() {
        // Debug: Show if auto-alignment is active
        if (autoAlignmentActive) {
            telemetry.addData("🔍 DEBUG Auto-Align", "Active - Timer: %.1fs", autoAlignmentTimer.seconds());
        }
        
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
        
        // Debug: Check VisionPortal state
        if (visionPortal == null) {
            telemetry.addData("❌ DEBUG", "VisionPortal is null");
            autoAlignmentActive = false;
            return;
        }
        
        telemetry.addData("🔍 DEBUG", "VisionPortal state: %s", visionPortal.getCameraState());
        
        try {
            currentDetections = aprilTag.getDetections();
            telemetry.addData("🔍 DEBUG", "Detections retrieved - Count: %d", 
                currentDetections != null ? currentDetections.size() : 0);
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
        
        // Look for the target tag
        for (AprilTagDetection detection : currentDetections) {
            if (detection != null && detection.ftcPose != null && detection.id == TARGET_TAG_ID) {
                targetFound = true;
                
                // Get bearing angle (detection angle) - this is what we want to minimize
                double bearingAngle = detection.ftcPose.bearing;  // Angle to tag in degrees
                double range = detection.ftcPose.range;           // Distance to tag
                
                telemetry.addData("🎯 Target Found", "Tag %d", TARGET_TAG_ID);
                telemetry.addData("🧭 Bearing Angle", "%.1f degrees", bearingAngle);
                telemetry.addData("📏 Range", "%.1f inches", range);
                
                // Calculate drive powers for bearing-based alignment
                double turnPower = 0;
                
                // Turn to minimize bearing angle
                if (Math.abs(bearingAngle) > BEARING_ALIGNMENT_TOLERANCE) {
                    turnPower = bearingAngle * BEARING_ALIGNMENT_POWER;
                    // Clamp to max power
                    turnPower = Math.max(-ALIGNMENT_TURN_POWER, 
                               Math.min(ALIGNMENT_TURN_POWER, turnPower));
                    
                    // Add minimum power if error is significant but calculated power is too small
                    double minTurnPower = 0.12;
                    if (Math.abs(bearingAngle) > BEARING_ALIGNMENT_TOLERANCE && Math.abs(turnPower) < minTurnPower) {
                        turnPower = Math.signum(turnPower) * minTurnPower;
                    }
                }
                
                // Check if aligned
                boolean aligned = Math.abs(bearingAngle) < BEARING_ALIGNMENT_TOLERANCE;
                
                if (aligned) {
                    // Perfect alignment achieved!
                    autoAlignmentActive = false;
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                    telemetry.addData("🎯 Auto-Alignment", "✅ PERFECT ALIGNMENT!");
                    telemetry.addData("🚀 READY", "Optimal position for shooting!");
                } else {
                    // Apply alignment movements
                    double frontLeftPower = turnPower;
                    double frontRightPower = -turnPower;
                    double backLeftPower = turnPower;
                    double backRightPower = -turnPower;
                    
                    leftFront.setPower(frontLeftPower);
                    rightFront.setPower(frontRightPower);
                    leftBack.setPower(backLeftPower);
                    rightBack.setPower(backRightPower);
                    
                    telemetry.addData("🎯 Auto-Alignment", "⚙️ ALIGNING...");
                    telemetry.addData("🔧 Turn Power", "%.3f", turnPower);
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
    
    // ========== BALL DETECTION AND INTAKE MANAGEMENT ==========
    
    /**
     * Read all color sensors and detect ball presence
     */
    private void readColorSensors() {
        // Read color sensor values
        NormalizedRGBA colorsIntake = colorSensorIntake.getNormalizedColors();
        NormalizedRGBA colorsFire = colorSensorFire.getNormalizedColors();
        NormalizedRGBA colorsStore = colorSensorStore.getNormalizedColors();
        
        // Update previous states for edge detection
        previousBallDetectedIntake = ballDetectedIntake;
        previousBallDetectedFire = ballDetectedFire;  
        previousBallDetectedStore = ballDetectedStore;
        
        // Detect balls using alpha channel (object presence)
        ballDetectedIntake = colorsIntake.alpha > BALL_DETECTION_THRESHOLD;
        ballDetectedFire = colorsFire.alpha > BALL_DETECTION_THRESHOLD;
        ballDetectedStore = colorsStore.alpha > BALL_DETECTION_THRESHOLD;
        
        // Update ball count based on sensor detection
        updateBallCount();
        
        // Check for intake safety stop (3 balls detected)
        checkIntakeSafetyStop();
        
        // Check for indexor stuck during intake operations
        checkIntakeIndexorStuck();
    }
    
    /**
     * Update ball count based on sensor readings and edge detection
     */
    private void updateBallCount() {
        // Count current balls detected across all sensors
        int currentDetectedBalls = 0;
        if (ballDetectedIntake) currentDetectedBalls++;
        if (ballDetectedFire) currentDetectedBalls++;
        if (ballDetectedStore) currentDetectedBalls++;
        
        // Use the maximum between current count and detected balls
        // This prevents count from decreasing too quickly during ball movement
        ballCount = Math.max(ballCount, currentDetectedBalls);
        
        // Detect new ball entering system (rising edge at intake sensor)
        if (ballDetectedIntake && !previousBallDetectedIntake) {
            // Ball entered - increment count only if under limit
            if (ballCount < MAX_BALLS_ALLOWED) {
                ballCount++;
                telemetry.addData("🎾 Ball Detected", "New ball entered! Count: %d", ballCount);
            }
        }
        
        // Detect ball leaving system (falling edge at fire sensor)
        if (!ballDetectedFire && previousBallDetectedFire) {
            // Ball exited - decrement count
            if (ballCount > 0) {
                ballCount--;
                telemetry.addData("🚀 Ball Fired", "Ball shot! Count: %d", ballCount);
            }
        }
        
        // Safety bounds checking
        ballCount = Math.max(0, Math.min(ballCount, MAX_BALLS_ALLOWED));
    }
    
    /**
     * Check if intake should be stopped due to 3 balls detected
     */
    private void checkIntakeSafetyStop() {
        // If all three sensors detect balls, stop intake and turn LED red
        if (ballDetectedIntake && ballDetectedFire && ballDetectedStore) {
            intakeSafetyStop = true;
            ballCount = MAX_BALLS_ALLOWED; // Ensure count is at maximum
            
            // Stop intake motor
            if (intakeRunning) {
                intake.setPower(0);
                intakeRunning = false;
                telemetry.addData("🛑 INTAKE STOP", "System FULL - 3 balls detected!");
            }
            
            // Turn speed light RED to indicate system is full
            speedLight.setPosition(LIGHT_RED_POSITION);
            
        } else {
            // Reset safety stop if not all sensors detect balls
            intakeSafetyStop = false;
        }
    }
    
    /**
     * Check for indexor stuck during intake operations and reverse conveyor if needed
     */
    private void checkIntakeIndexorStuck() {
        // Only check for stuck detection during intake or indexor operations
        if (!intakeRunning && !indexorRunningToPosition) {
            return;
        }
        
        int currentPosition = indexor.getCurrentPosition();
        
        // Check if indexor has made sufficient progress
        if (Math.abs(currentPosition - indexorLastPosition) > INTAKE_STUCK_PROGRESS_THRESHOLD) {
            // Indexor is moving - reset stuck detection
            stuckDetectionTimer.reset();
            indexorLastPosition = currentPosition;
            return;
        }
        
        // Check if stuck timeout has been reached
        if (stuckDetectionTimer.seconds() > INTAKE_STUCK_DETECTION_TIMEOUT) {
            // Indexor is stuck - reverse conveyor to clear jam
            telemetry.addData("⚠️ JAM DETECTED", "Reversing conveyor to clear jam...");
            
            // Reverse conveyor for specified duration
            conveyor.setPower(-CONVEYOR_POWER);
            
            // Stop intake during jam clearing
            if (intakeRunning) {
                intake.setPower(0);
                intakeRunning = false;
            }
            
            // Reset stuck detection timer
            stuckDetectionTimer.reset();
            
            // After a delay, restart normal operation
            try {
                Thread.sleep((long)(INTAKE_CONVEYOR_REVERSE_DURATION * 1000));
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            
            // Resume normal conveyor operation if intake was running
            conveyor.setPower(CONVEYOR_POWER);
            
            telemetry.addData("✅ JAM CLEARED", "Resuming normal operation");
        }
    }
    
    /**
     * Enhanced toggleIntakeSystem with ball count management
     */
    private void toggleIntakeSystemWithBallCount() {
        // Check if intake is blocked due to safety stop
        if (intakeSafetyStop) {
            telemetry.addData("🛑 INTAKE BLOCKED", "System FULL! Fire balls to continue");
            return;
        }
        
        if (!intakeRunning) {
            // Start intake system
            intake.setPower(INTAKE_POWER);
            conveyor.setPower(CONVEYOR_POWER);
            intakeRunning = true;
            
            // Reset stuck detection timer
            stuckDetectionTimer.reset();
            indexorLastPosition = indexor.getCurrentPosition();
            
            telemetry.addData("✅ Intake", "STARTED - Ball count: %d/%d", ballCount, MAX_BALLS_ALLOWED);
        } else {
            // Stop intake system
            intake.setPower(0);
            conveyor.setPower(0);
            intakeRunning = false;
            telemetry.addData("⏹️ Intake", "STOPPED - Ball count: %d/%d", ballCount, MAX_BALLS_ALLOWED);
        }
    }
}