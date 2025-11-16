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
import android.util.Size;

@Config
@TeleOp(name = "TeleOpDECODESimple2", group = "TeleOp")
public class TeleOpDECODESimple2 extends LinearOpMode {
    
    // Declare motors
    private DcMotorEx indexor;
    private DcMotorEx intake;
    private DcMotorEx conveyor;
    private DcMotorEx shooter;
    
    // Declare servos
    private CRServo shooterServo;
    private Servo triggerServo;
    
    // Declare color sensor for intake ball detection
    private NormalizedColorSensor colorSensorIntake;
    
    // Declare color sensor for middle position
    private NormalizedColorSensor colorSensorMiddle;
    
    // Declare mecanum drive motors
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    
    // AprilTag detection system
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    
    // Variables to track button states
    private boolean previousA1 = false;  // Gamepad1 A button (intake)
    private boolean previousB1 = false;  // Gamepad1 B button (shooter speed 1300)
    private boolean previousY1 = false;  // Gamepad1 Y button (shooter speed 1600)
    private boolean previousB2 = false;  // Gamepad2 B button (trigger)
    private boolean previousX2 = false;  // Gamepad2 X button (advance indexer)
    
    // Ball detection variables
    private boolean ballDetectedIntake = false;
    private boolean previousBallDetectedIntake = false;
    private boolean ballDetectedMiddle = false;
    
    // Indexor control variables
    private double indexorLastSuccessfulPosition = 0.0;  // Last successful indexor position
    private boolean indexorMoving = false;
    private ElapsedTime indexorTimer = new ElapsedTime();
    private int indexorStartPosition = 0;
    
    // Shooter variables
    private boolean shooterRunning = false;
    private double currentShooterVelocity = 1300;  // Default velocity
    
    // Trigger sequence variables
    private boolean triggerSequenceActive = false;
    private ElapsedTime triggerTimer = new ElapsedTime();
    private boolean triggerInFirePosition = false;
    
    // Motor power settings
    public static final double INTAKE_POWER = 0.8;
    public static final double CONVEYOR_POWER = 1.0;
    public static final double INDEXOR_POWER = 0.4;
    public static final double SHOOTER_SERVO_POWER = 1.0;
    
    // Indexor position settings
    public static final double INDEXOR_TICKS_PER_REVOLUTION = 537.7;  // goBILDA 312 RPM motor
    public static final double INDEXOR_TICKS_PER_120_DEGREES = INDEXOR_TICKS_PER_REVOLUTION / 3.0;  // 179.23 ticks per 120°
    
    // Shooter velocity settings
    public static final double SHOOTER_TARGET_VELOCITY_BUTTON = 1300;  // Button-based velocity
    public static final double SHOOTER_TARGET_VELOCITY_DISTANCE = 1500; // Distance-based velocity
    
    // Color sensor settings
    public static final double COLOR_SENSOR_GAIN = 15.0;
    public static final double BALL_DETECTION_THRESHOLD = 0.15;
    
    // Trigger servo positions
    public static final double TRIGGER_FIRE = 0.05;     // Fire position (27.0 degrees)
    public static final double TRIGGER_HOME = 0.5;     // Home position (104.4 degrees)
    public static final double TRIGGER_FIRE_DURATION = 0.5;  // Fire duration in seconds
    
    // Indexor stuck detection
    public static final double INDEXOR_STUCK_TIMEOUT = 0.5;  // 0.5 seconds as specified
    public static final int INDEXOR_STUCK_THRESHOLD = 10;    // Minimum movement required
    
    // AprilTag settings
    public static final int TARGET_TAG_ID = 20;
    public static final double OPTIMAL_SHOOTING_DISTANCE = 24.0; // inches
    
    // Mecanum drive settings
    public static final double DRIVE_SPEED_MULTIPLIER = 0.8;
    public static final double STRAFE_SPEED_MULTIPLIER = 0.8;
    public static final double TURN_SPEED_MULTIPLIER = 0.6;
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeMotors();
        initializeAprilTag();
        
        // Preserve indexor position from autonomous - do not reset
        indexorLastSuccessfulPosition = indexor.getCurrentPosition();
        
        telemetry.addData("Status", "TeleOpDECODESimple2 - Initialized");
        telemetry.addData("Indexor Position", "Preserved: %.1f ticks", indexorLastSuccessfulPosition);
        telemetry.addData("", "");
        telemetry.addData("GAMEPAD1 CONTROLS:", "");
        telemetry.addData("A", "Intake Function");
        telemetry.addData("B", "Toggle Shooter Speed 1300");
        telemetry.addData("Y", "Toggle Shooter Speed 1600");
        telemetry.addData("Left Stick", "Drive/Strafe");
        telemetry.addData("Right Stick X", "Turn");
        telemetry.addData("", "");
        telemetry.addData("GAMEPAD2 CONTROLS:", "");
        telemetry.addData("X", "Advance Indexer");
        telemetry.addData("B", "Trigger Function");
        telemetry.addData("Left Stick -Y", "Outtake Function");
        telemetry.update();
        
        waitForStart();
        
        // Main control loop
        while (opModeIsActive()) {
            readColorSensors();
            handleGamepad1Controls();
            handleGamepad2Controls();
            handleIndexorStuckDetection();
            handleTriggerSequence();
            handleMecanumDrive();
            updateTelemetry();
            sleep(20);
        }
    }
    
    private void initializeMotors() {
        // Initialize motors
        indexor = hardwareMap.get(DcMotorEx.class, "indexor");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        conveyor = hardwareMap.get(DcMotorEx.class, "conveyor");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        
        // Initialize servos
        shooterServo = hardwareMap.get(CRServo.class, "shooterServo");
        triggerServo = hardwareMap.get(Servo.class, "triggerServo");
        
        // Initialize mecanum drive motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        
        // Set motor directions (from original)
        indexor.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        conveyor.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        
        // Set servo directions
        shooterServo.setDirection(DcMotorSimple.Direction.REVERSE);
        triggerServo.setDirection(Servo.Direction.FORWARD);
        
        // Initialize trigger servo to home position
        triggerServo.setPosition(TRIGGER_HOME);
        
        // Initialize color sensor
        colorSensorIntake = hardwareMap.get(NormalizedColorSensor.class, "colorSensorEntry");
        colorSensorIntake.setGain((float)COLOR_SENSOR_GAIN);
        
        // Initialize middle color sensor
        colorSensorMiddle = hardwareMap.get(NormalizedColorSensor.class, "colorSensorMiddle");
        colorSensorMiddle.setGain((float)COLOR_SENSOR_GAIN);
        
        // Enable LED light if available
        if (colorSensorIntake instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorIntake).enableLight(true);
        }
        
        // Enable LED light for middle sensor if available
        if (colorSensorMiddle instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorMiddle).enableLight(true);
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
        
        // Set motor modes
        indexor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Preserve position from auto
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Configure drive motors
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set zero power behavior for drive
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    private void initializeAprilTag() {
        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(1156.544, 1156.544, 640.0, 360.0)
                .build();

        try {
            // Create the vision portal
            VisionPortal.Builder builder = new VisionPortal.Builder();
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            builder.setCameraResolution(new Size(1280, 720));
            builder.enableLiveView(true);
            builder.setAutoStopLiveView(false);
            builder.addProcessor(aprilTag);
            
            visionPortal = builder.build();
            
        } catch (Exception e) {
            telemetry.addData("❌ AprilTag Error", "Failed to initialize: %s", e.getMessage());
            visionPortal = null;
            aprilTag = null;
        }
    }
    
    private void readColorSensors() {
        // Read intake color sensor
        NormalizedRGBA colorsIntake = colorSensorIntake.getNormalizedColors();
        
        // Update previous state
        previousBallDetectedIntake = ballDetectedIntake;
        
        // Detect ball using alpha channel
        ballDetectedIntake = colorsIntake.alpha > BALL_DETECTION_THRESHOLD;
        
        // Read middle color sensor
        NormalizedRGBA colorsMiddle = colorSensorMiddle.getNormalizedColors();
        
        // Detect ball in middle position
        ballDetectedMiddle = colorsMiddle.alpha > BALL_DETECTION_THRESHOLD;
    }
    
    private void handleGamepad1Controls() {
        // Get current button states for gamepad1
        boolean currentA1 = gamepad1.a;
        boolean currentB1 = gamepad1.b;
        boolean currentY1 = gamepad1.y;
        
        // Handle A button - Intake Function
        if (currentA1 && !previousA1) {
            intakeFunction();
        }
        
        // Handle B button - Toggle Shooter Speed 1300
        if (currentB1 && !previousB1) {
            toggleShooterSpeed(1300);
        }
        
        // Handle Y button - Toggle Shooter Speed 1600
        if (currentY1 && !previousY1) {
            toggleShooterSpeed(1600);
        }
        
        // Update previous states
        previousA1 = currentA1;
        previousB1 = currentB1;
        previousY1 = currentY1;
    }
    
    private void handleGamepad2Controls() {
        // Handle joystick -Y - Outtake Function
        double joystickY = -gamepad2.left_stick_y;
        
        if (joystickY < -0.1) {  // Negative Y (joystick pushed up)
            outtakeFunction();
        } else {
            // Stop outtake when joystick released
            if (Math.abs(intake.getPower()) > 0 && intake.getPower() < 0) {
                stopOuttake();
            }
        }
        
        // Get current button states for gamepad2
        boolean currentX2 = gamepad2.x;
        boolean currentB2 = gamepad2.b;
        
        // Handle X button - Advance Indexer
        if (currentX2 && !previousX2) {
            advanceIndexer();
        }
        
        // Handle B button - Trigger Function
        if (currentB2 && !previousB2) {
            triggerFunction();
        }
        
        // Update previous states
        previousX2 = currentX2;
        previousB2 = currentB2;
    }
    
    /**
     * Function Intake (gamepad1 A button)
     * 1) Run the intake wheels forward
     * 2) Run the conveyor forward  
     * 3) Advance the indexer forward when ball detected in the intake sensor
     * 4) Stop intake when middle sensor detects a ball
     */
    private void intakeFunction() {
        boolean intakeRunning = Math.abs(intake.getPower()) > 0.1;
        
        if (!intakeRunning) {
            // Check if middle sensor already has a ball before starting intake
            if (ballDetectedMiddle) {
                telemetry.addData("⚠️ Intake", "Cannot start - Middle sensor FULL");
                telemetry.addData("Middle Sensor", "🔴 Ball detected");
                telemetry.update();
                return;
            }
            
            // Start intake
            intake.setPower(INTAKE_POWER);           // 1) Run intake wheels forward
            conveyor.setPower(CONVEYOR_POWER);       // 2) Run conveyor forward
            
            telemetry.addData("✅ Intake Function", "STARTED");
            telemetry.addData("Intake Power", "%.1f", INTAKE_POWER);
            telemetry.addData("Conveyor Power", "%.1f", CONVEYOR_POWER);
            telemetry.addData("Ball Detection", "Monitoring for auto-advance and auto-stop");
        } else {
            // Stop intake
            intake.setPower(0);
            conveyor.setPower(0);
            
            telemetry.addData("⏹️ Intake Function", "STOPPED");
        }
        
        // 4) Auto-stop intake if middle sensor detects a ball while running
        if (intakeRunning && ballDetectedMiddle && !indexorMoving) {
            intake.setPower(0);
            conveyor.setPower(0);
            telemetry.addData("🛑 Auto Stop", "Middle sensor FULL - intake stopped");
            telemetry.addData("Middle Sensor", "🔴 Ball detected");
            telemetry.addData("Status", "Ready for next operation!");
        }
        
        // 3) Advance the indexer forward when ball detected in the intake sensor
        // Check for rising edge of ball detection during intake
        if (ballDetectedIntake && !previousBallDetectedIntake) {
            // Ball detected - check if intake is running
            boolean currentIntakeRunning = Math.abs(intake.getPower()) > 0.1 && intake.getPower() > 0;
            
            if (currentIntakeRunning && !indexorMoving && !ballDetectedMiddle) {
                // Auto-advance indexer when ball detected during intake (if middle not full)
                advanceIndexer();
                telemetry.addData("🎾 Auto Advance", "Ball detected - advancing indexer");
            }
        }
        
        telemetry.update();
    }
    
    /**
     * Function Outtake (gamepad2 joystick -Y)
     * 1) Run the intake wheels rearward
     * 2) Run the conveyor rearward
     * 3) Hold the indexer
     */
    private void outtakeFunction() {
        intake.setPower(-INTAKE_POWER);          // 1) Run intake wheels rearward
        conveyor.setPower(-CONVEYOR_POWER);      // 2) Run conveyor rearward
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // 3) Hold the indexer
        indexor.setPower(0);
        
        telemetry.addData("🔄 Outtake Function", "ACTIVE");
        telemetry.addData("Intake", "REVERSE at %.1f", -INTAKE_POWER);
        telemetry.addData("Conveyor", "REVERSE at %.1f", -CONVEYOR_POWER);
        telemetry.addData("Indexor", "HOLDING (BRAKE mode)");
    }
    
    private void stopOuttake() {
        intake.setPower(0);
        conveyor.setPower(0);
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        indexor.setPower(0);  // Ensure indexer is set to float mode when outtake stops
        
        telemetry.addData("⏹️ Outtake Function", "STOPPED");
        telemetry.addData("Indexor", "Set to FLOAT mode");
    }
    
    /**
     * Toggle Shooter Speed (gamepad1 B/Y buttons)
     * B button: Toggle 1300 ticks/sec
     * Y button: Toggle 1600 ticks/sec
     * If shooter is off or running at different speed, start at specified speed
     * If shooter is already running at specified speed, stop shooter
     */
    private void toggleShooterSpeed(double velocity) {
        if (shooterRunning && Math.abs(currentShooterVelocity - velocity) < 50) {
            // Shooter is already running at this speed - stop it
            shooter.setVelocity(0);
            shooterServo.setPower(0);
            shooterRunning = false;
            currentShooterVelocity = 0;
            
            telemetry.addData("⏹️ Shooter STOPPED", "Was running at %.0f ticks/sec", velocity);
            telemetry.addData("Shooter Status", "OFF");
        } else {
            // Shooter is off or running at different speed - start at specified velocity
            shooter.setVelocity(velocity);
            shooterServo.setPower(SHOOTER_SERVO_POWER);
            shooterRunning = true;
            currentShooterVelocity = velocity;
            
            telemetry.addData("🎯 Shooter STARTED", "%.0f ticks/sec", velocity);
            telemetry.addData("Shooter Status", "RUNNING");
        }
        telemetry.update();
    }
    
    /**
     * Function Advance Indexer
     * 1) Advance indexer to next 120°
     * 2) Use mod function to calculate next position
     * 3) Only advance the indexer to next level based on successful previous advancement
     */
    private void advanceIndexer() {
        // Check if indexer is already moving
        if (indexorMoving) {
            telemetry.addData("⚠️ Indexer", "Already moving - please wait");
            return;
        }
        
        // Get current actual position for telemetry only
        double currentActualPosition = (double) indexor.getCurrentPosition();
        double currentRemainder = currentActualPosition % INDEXOR_TICKS_PER_120_DEGREES;
        
        // Next position is always increment of 120 degrees from previous successful position
        double nextPosition = indexorLastSuccessfulPosition + INDEXOR_TICKS_PER_120_DEGREES;
        
        // Calculate step information for telemetry
        int lastSuccessfulStep = (int) Math.round(indexorLastSuccessfulPosition / INDEXOR_TICKS_PER_120_DEGREES) % 3;
        int nextStep = (int) Math.round(nextPosition / INDEXOR_TICKS_PER_120_DEGREES) % 3;
        
        // Set target position
        indexor.setTargetPosition((int) Math.round(nextPosition));
        indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexor.setPower(INDEXOR_POWER);
        
        // Start movement tracking
        indexorMoving = true;
        indexorTimer.reset();
        indexorStartPosition = indexor.getCurrentPosition();
        
        telemetry.addData("🎯 Advance Indexer", "Moving to next 120°");
        telemetry.addData("Current Actual", "%.1f ticks (remainder: %.1f)", currentActualPosition, currentRemainder);
        telemetry.addData("Last Successful", "%.1f ticks (step %d)", indexorLastSuccessfulPosition, lastSuccessfulStep);
        telemetry.addData("Target Position", "%.1f ticks (step %d)", nextPosition, nextStep);
        telemetry.addData("120° Advancement", "Step %d → Step %d", lastSuccessfulStep, nextStep);
        telemetry.update();
    }
    
    /**
     * Handle indexer stuck detection
     * if indexer stuck for 0.5sec put the indexer in float mode
     * Only update successful position when movement completes successfully to exact target
     */
    private void handleIndexorStuckDetection() {
        if (!indexorMoving) {
            return;
        }
        
        // Check if indexer reached target
        if (!indexor.isBusy()) {
            // Check if indexer actually reached the target position (within tolerance)
            int currentPosition = indexor.getCurrentPosition();
            int targetPosition = indexor.getTargetPosition();
            int positionError = Math.abs(currentPosition - targetPosition);
            
            if (positionError <= 15) {  // Within 15 ticks tolerance for successful 120° advancement
                // Movement completed successfully to target - update successful position
                indexorLastSuccessfulPosition = targetPosition;
                telemetry.addData("✅ Indexer", "Successfully advanced to %.1f", indexorLastSuccessfulPosition);
            } else {
                // Movement completed but not at target position - DO NOT update successful position
                telemetry.addData("⚠️ Indexer", "Reached end but not at target (error: %d ticks)", positionError);
                telemetry.addData("Target", "%d, Actual: %d", targetPosition, currentPosition);
                telemetry.addData("Keeping Previous", "Successful position: %.1f", indexorLastSuccessfulPosition);
            }
            
            indexorMoving = false;
            indexor.setPower(0);
            return;
        }
        
        // Check for stuck condition (0.5 seconds as specified)
        if (indexorTimer.seconds() > INDEXOR_STUCK_TIMEOUT) {
            int currentPosition = indexor.getCurrentPosition();
            int movement = Math.abs(currentPosition - indexorStartPosition);
            
            if (movement < INDEXOR_STUCK_THRESHOLD) {
                // Indexer is stuck - put in float mode
                // DO NOT update indexorLastSuccessfulPosition - keep previous successful position
                indexor.setPower(0);
                indexor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                indexorMoving = false;
                
                telemetry.addData("⚠️ Indexer STUCK", "Put in FLOAT mode");
                telemetry.addData("Movement", "%d ticks in %.1f seconds", movement, indexorTimer.seconds());
                telemetry.addData("Keeping Previous", "Successful position: %.1f", indexorLastSuccessfulPosition);
                telemetry.update();
            }
        }
    }
    
    private double determineShooterVelocity() {
        // Check for AprilTag distance-based velocity
        if (visionPortal != null && aprilTag != null) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            
            for (AprilTagDetection detection : detections) {
                if (detection.id == TARGET_TAG_ID && detection.ftcPose != null) {
                    double distance = detection.ftcPose.range;
                    
                    if (Math.abs(distance - OPTIMAL_SHOOTING_DISTANCE) < 6.0) {
                        // Close to optimal distance - use distance-based velocity
                        return SHOOTER_TARGET_VELOCITY_DISTANCE;
                    }
                }
            }
        }
        
        // Default to button-based velocity
        return SHOOTER_TARGET_VELOCITY_BUTTON;
    }
    
    /**
     * Function Trigger
     * 1) Check shooter is running at target velocity
     * 2) Put the servo in fire position for 0.5 sec
     * 3) Advance indexer to next position
     */
    private void triggerFunction() {
        if (triggerSequenceActive) {
            telemetry.addData("⚠️ Trigger", "Sequence already active");
            return;
        }
        
        // 1) Check if shooter is running at target velocity
        if (!shooterRunning) {
            telemetry.addData("⚠️ Trigger", "Shooter not running - use gamepad1 B/Y to set speed first");
            telemetry.update();
            return;
        }
        
        // 2) Start trigger sequence - move to fire position
        triggerServo.setPosition(TRIGGER_FIRE);
        triggerSequenceActive = true;
        triggerInFirePosition = true;
        triggerTimer.reset();
        
        telemetry.addData("🎯 Trigger Function", "SEQUENCE STARTED");
        telemetry.addData("Shooter Status", "RUNNING at %.0f ticks/sec", currentShooterVelocity);
        telemetry.addData("Trigger Position", "FIRE (%.1f seconds)", TRIGGER_FIRE_DURATION);
        telemetry.update();
    }
    
    private void handleTriggerSequence() {
        if (!triggerSequenceActive) {
            return;
        }
        
        // Check if fire duration is complete
        if (triggerInFirePosition && triggerTimer.seconds() >= TRIGGER_FIRE_DURATION) {
            // Return trigger to home position
            triggerServo.setPosition(TRIGGER_HOME);
            triggerInFirePosition = false;
            triggerTimer.reset();
            
            telemetry.addData("🏠 Trigger", "Returned to HOME position");
            
            // Add 0.25 second delay before advancing indexer
            try {
                Thread.sleep(250);  // 0.25 second delay
            } catch (InterruptedException e) {
                // Handle interruption gracefully
                Thread.currentThread().interrupt();
            }
            
            // 3) Advance indexer to next position
            advanceIndexer();
            
            // End sequence
            triggerSequenceActive = false;
            
            telemetry.addData("✅ Trigger Sequence", "COMPLETE");
            telemetry.update();
        }
    }
    
    private void handleMecanumDrive() {
        // Standard mecanum drive
        double drive = -gamepad1.left_stick_y * DRIVE_SPEED_MULTIPLIER;
        double strafe = gamepad1.left_stick_x * STRAFE_SPEED_MULTIPLIER;
        double turn = -gamepad1.right_stick_x * TURN_SPEED_MULTIPLIER;
        
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
        
        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftBack.setPower(backLeftPower);
        rightBack.setPower(backRightPower);
    }
    
    private void updateTelemetry() {
        telemetry.addData("Status", "TeleOpDECODESimple2 - RUNNING");
        telemetry.addData("", "");
        
        // Ball detection status
        String ballStatusIntake = ballDetectedIntake ? "🔴 DETECTED" : "⚪ CLEAR";
        String ballStatusMiddle = ballDetectedMiddle ? "🔴 DETECTED" : "⚪ CLEAR";
        telemetry.addData("Intake Sensor", "%s", ballStatusIntake);
        telemetry.addData("Middle Sensor", "%s", ballStatusMiddle);
        
        // Check for automatic indexer advancement based on ball detection
        checkAutomaticIndexerAdvance();
        
        // Motor status
        telemetry.addData("Intake Power", "%.2f", intake.getPower());
        telemetry.addData("Conveyor Power", "%.2f", conveyor.getPower());
        telemetry.addData("Indexor Power", "%.2f", indexor.getPower());
        
        // Indexer position
        int currentPosition = indexor.getCurrentPosition();
        double logicalPosition = (indexorLastSuccessfulPosition % INDEXOR_TICKS_PER_REVOLUTION) / INDEXOR_TICKS_PER_120_DEGREES;
        telemetry.addData("Indexor Position", "Current: %d, Last Successful: %.1f", 
            currentPosition, indexorLastSuccessfulPosition);
        telemetry.addData("Logical Position", "%.1f (%.0f degrees)", logicalPosition, logicalPosition * 120);
        
        // Shooter status
        if (shooterRunning) {
            telemetry.addData("Shooter", "RUNNING at %.0f ticks/sec", currentShooterVelocity);
            telemetry.addData("Current Velocity", "%.0f ticks/sec", shooter.getVelocity());
        } else {
            telemetry.addData("Shooter", "STOPPED");
        }
        
        // Trigger status
        if (triggerSequenceActive) {
            if (triggerInFirePosition) {
                double timeLeft = TRIGGER_FIRE_DURATION - triggerTimer.seconds();
                telemetry.addData("Trigger Sequence", "FIRE position (%.1fs left)", timeLeft);
            } else {
                telemetry.addData("Trigger Sequence", "Returning to HOME");
            }
        } else {
            double currentTriggerPos = triggerServo.getPosition();
            boolean inFirePos = Math.abs(currentTriggerPos - TRIGGER_FIRE) < 0.05;
            telemetry.addData("Trigger Position", "%s (%.2f)", 
                inFirePos ? "FIRE" : "HOME", currentTriggerPos);
        }
        
        telemetry.addData("", "");
        telemetry.addData("Drive Controls", "Left stick: Drive/Strafe, Right stick X: Turn");
        
        telemetry.update();
    }
    
    /**
     * Check for automatic indexer advancement when ball is detected during intake
     * Part of Intake Function requirement: "3) Advance the indexer forward when ball detected in the intake sensor"
     */
    private void checkAutomaticIndexerAdvance() {
        // Check for rising edge of ball detection during intake
        if (ballDetectedIntake && !previousBallDetectedIntake) {
            // Ball detected - check if intake is running
            boolean intakeRunning = Math.abs(intake.getPower()) > 0.1 && intake.getPower() > 0;
            
            if (intakeRunning && !indexorMoving) {
                // Auto-advance indexer when ball detected during intake
                advanceIndexer();
                telemetry.addData("🎾 Auto Advance", "Ball detected - advancing indexer");
            }
        }
    }
}