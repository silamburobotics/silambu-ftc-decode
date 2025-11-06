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
    
    // Trigger servo management
    private boolean manualTriggerControl = false; // Track if trigger is under manual control
    private ElapsedTime triggerManualTimer = new ElapsedTime(); // Timer for manual control timeout
    private boolean triggerAutoFireSequence = false; // Track if auto fire sequence is active
    private ElapsedTime triggerFireTimer = new ElapsedTime(); // Timer for fire sequence
    
    // Indexor stuck detection variables
    private ElapsedTime indexorTimer = new ElapsedTime();
    private int lastIndexorPosition = 0;
    private boolean indexorStuckDetectionActive = false;
    
    // Indexor position control variables
    private boolean indexorRunningToPosition = false;
    
    // Manual indexor control variables
    private boolean manualIndexorControl = false;             // Track if indexor is under manual control
    
    // Motor power settings
    public static final double INTAKE_POWER = 0.8;
    public static final double CONVEYOR_POWER = 1.0;
    public static final double AUTO_INDEXOR_POWER = 0.1;      // Power for automatic indexor movement
    public static final double MANUAL_INDEXOR_POWER = 0.3;    // Power for manual indexor control via joystick
    public static final double SHOOTER_POWER = 1.0;
    public static final double SHOOTER_SERVO_POWER = 1.0;     // Positive for forward direction
    
    // Indexor position settings
    public static final int INDEXOR_TICKS = 179;              // goBILDA 312 RPM motor: 120 degrees = 179 ticks
    
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
    
    // Mecanum drive settings
    public static final double DRIVE_SPEED_MULTIPLIER = 0.8;  // Max drive speed (0.0 to 1.0)
    public static final double STRAFE_SPEED_MULTIPLIER = 0.8; // Max strafe speed (0.0 to 1.0)
    public static final double TURN_SPEED_MULTIPLIER = 0.6;   // Max turn speed (0.0 to 1.0)
    
    // Manual control settings
    public static final double JOYSTICK_DEADZONE = 0.1;       // Deadzone for joystick input
    
    @Override
    public void runOpMode() {
        // Initialize motors
        initializeMotors();
        
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "DECODE Simple + Intake - Initialized");
        telemetry.addData("=== GAMEPAD 1 (DRIVER) ===", "");
        telemetry.addData("A Button", "Intake + Conveyor + Indexor");
        telemetry.addData("Left Stick", "Drive/Strafe");
        telemetry.addData("Right Stick X", "Turn");
        telemetry.addData("=== GAMEPAD 2 (OPERATOR) ===", "");
        telemetry.addData("X Button", "Indexor + Conveyor (%d ticks)", INDEXOR_TICKS);
        telemetry.addData("Y Button", "Shooter + Servo (Auto-stops Intake)");
        telemetry.addData("B Button", "Auto Fire (Fire → Home)");
        telemetry.addData("", "");
        telemetry.addData("Drive Speed", "%.0f%% max", DRIVE_SPEED_MULTIPLIER * 100);
        telemetry.addData("Strafe Speed", "%.0f%% max", STRAFE_SPEED_MULTIPLIER * 100);
        telemetry.addData("Turn Speed", "%.0f%% max", TURN_SPEED_MULTIPLIER * 100);
        telemetry.addData("Note", "Driver/Operator Split Controls");
        telemetry.update();
        
        waitForStart();
        
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            handleControllerInputs();
            handleManualIndexorControl();
            handleMecanumDrive();
            checkIndexorStuck();
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
    
    private void handleControllerInputs() {
        // Get current button states for gamepad1 (driver)
        boolean currentA = gamepad1.a;
        boolean currentX1 = gamepad1.x;
        boolean currentY1 = gamepad1.y;
        boolean currentB1 = gamepad1.b;
        
        // Get current button states for gamepad2 (operator)
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
    
    private void startIndexor() {
        // Check if indexor is already running
        boolean indexorRunning = Math.abs(indexor.getPower()) > 0.1;
        
        if (indexorRunning) {
            // Stop indexor
            indexor.setPower(0);
            indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            indexor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            indexorStuckDetectionActive = false;
            indexorRunningToPosition = false;
            
            telemetry.addData("Indexor", "STOPPED");
        } else {
            // Start indexor with automatic power
            indexor.setPower(AUTO_INDEXOR_POWER);
            indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            startIndexorStuckDetection();
            
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
            // STEP 1: Stop intake system first for safety and performance
            boolean intakeWasRunning = Math.abs(intake.getPower()) > 0.1;
            if (intakeWasRunning) {
                intake.setPower(0);
                conveyor.setPower(0);
                indexor.setPower(0);
                telemetry.addData("🛑 INTAKE", "Auto-stopped for shooter startup");
            }
            
            // STEP 2: Start shooter with velocity control and servo
            shooter.setVelocity(SHOOTER_TARGET_VELOCITY);
            shooterServo.setPower(SHOOTER_SERVO_POWER);
            shooterIntentionallyRunning = true;
            currentShooterTargetVelocity = SHOOTER_TARGET_VELOCITY;
            
            telemetry.addData("Shooter System", "RUNNING at %.0f ticks/sec", SHOOTER_TARGET_VELOCITY);
            if (intakeWasRunning) {
                telemetry.addData("💡 Note", "Intake was stopped automatically");
            }
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
            
            telemetry.addData("Shooter System", "STOPPED");
        } else {
            // STEP 1: Stop intake system first for safety and performance
            boolean intakeWasRunning = Math.abs(intake.getPower()) > 0.1;
            if (intakeWasRunning) {
                intake.setPower(0);
                conveyor.setPower(0);
                indexor.setPower(0);
                telemetry.addData("🛑 INTAKE", "Auto-stopped for shooter startup");
            }
            
            // STEP 2: Start shooter with velocity control and servo at configurable speed
            shooter.setVelocity(SHOOTER_B_BUTTON_VELOCITY);
            shooterServo.setPower(SHOOTER_SERVO_POWER);
            shooterIntentionallyRunning = true;
            currentShooterTargetVelocity = SHOOTER_B_BUTTON_VELOCITY;
            
            // Reset stabilization tracking
            shooterStabilizationTimer.reset();
            shooterSpeedStable = false;
            
            telemetry.addData("Shooter System", "RUNNING at %.0f ticks/sec", SHOOTER_B_BUTTON_VELOCITY);
            telemetry.addData("Status", "🟡 Stabilizing speed...");
            if (intakeWasRunning) {
                telemetry.addData("💡 Note", "Intake was stopped automatically");
            }
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
            
            telemetry.addData("Shooter System", "STOPPED");
        } else {
            // STEP 1: Stop intake system first for safety and performance
            boolean intakeWasRunning = Math.abs(intake.getPower()) > 0.1;
            if (intakeWasRunning) {
                intake.setPower(0);
                conveyor.setPower(0);
                indexor.setPower(0);
                telemetry.addData("🛑 INTAKE", "Auto-stopped for shooter startup");
            }
            
            // STEP 2: Start shooter with velocity control and servo at 1300 ticks/sec
            shooter.setVelocity(1300);
            shooterServo.setPower(SHOOTER_SERVO_POWER);
            shooterIntentionallyRunning = true;
            currentShooterTargetVelocity = 1300;
            
            // Reset stabilization tracking
            shooterStabilizationTimer.reset();
            shooterSpeedStable = false;
            
            telemetry.addData("Shooter System", "RUNNING at 1300 ticks/sec");
            telemetry.addData("Status", "🟡 Stabilizing speed...");
            if (intakeWasRunning) {
                telemetry.addData("💡 Note", "Intake was stopped automatically");
            }
        }
        telemetry.update();
    }
    
    private void monitorShooterSpeed() {
        if (!shooterIntentionallyRunning) {
            shooterSpeedStable = false;
            return;
        }
        
        double currentVelocity = shooter.getVelocity();
        double speedError = Math.abs(currentVelocity - currentShooterTargetVelocity);
        double stabilizationTime = shooterStabilizationTimer.seconds();
        
        // Check if speed is within tolerance
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
            
            telemetry.addData("🏠 AUTO RETURN", "Trigger returned to HOME position (137°)");
            telemetry.addData("✅ COMPLETE", "Fire sequence finished");
            telemetry.update();
        }
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
            telemetry.addData("Target Velocity", "%.0f ticks/sec", currentShooterTargetVelocity);
            telemetry.addData("Current Velocity", "%.0f ticks/sec", shooter.getVelocity());
            double speedPercentage = shooter.getVelocity() / currentShooterTargetVelocity * 100;
            telemetry.addData("Speed Accuracy", "%.1f%%", speedPercentage);
            
            // Speed stabilization status
            double stabilizationTime = shooterStabilizationTimer.seconds();
            double speedError = Math.abs(shooter.getVelocity() - currentShooterTargetVelocity);
            
            if (shooterSpeedStable) {
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
                manualIndexorControl = true;
            }
            
            // Set manual indexor power (slow movement)
            double indexorPower = joystickY * MANUAL_INDEXOR_POWER;
            indexor.setPower(indexorPower);
            
            // Start conveyor when joystick is active
            conveyor.setPower(CONVEYOR_POWER);
            
            // Add telemetry for manual control
            telemetry.addData("🎮 Manual Indexor", "Active - Left Joystick");
            telemetry.addData("Joystick Y", "%.3f", joystickY);
            telemetry.addData("Indexor Power", "%.2f (%.0f%%)", indexorPower, indexorPower * 100);
            telemetry.addData("Conveyor", "RUNNING at %.2f power", CONVEYOR_POWER);
            telemetry.addData("Control Mode", "MANUAL - Slow Movement");
            
        } else if (manualIndexorControl) {
            // Joystick released - stop indexor, conveyor and return to automatic mode
            indexor.setPower(0);
            conveyor.setPower(0);
            manualIndexorControl = false;
            telemetry.addData("🎮 Manual Indexor", "Released - Auto Mode");
            telemetry.addData("Conveyor", "STOPPED - Manual Released");
        }
    }
}