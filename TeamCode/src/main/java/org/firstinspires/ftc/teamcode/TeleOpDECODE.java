package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

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
    
    // AprilTag detection
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    
    // Variables to track button states
    private boolean previousX = false;
    private boolean previousA = false;
    private boolean previousY = false;
    private boolean previousB = false;
    private boolean isAlignedToTag = false; // Track if robot is aligned to AprilTag
    
    // Motor power settings
    public static final double INTAKE_POWER = 0.8;
    public static final double SHOOTER_POWER = 1.0;
    public static final double CONVEYOR_POWER = 1.0;
    public static  int INDEXOR_TICKS = 179; // goBILDA 312 RPM motor: 120 degrees = 179 ticks
    
    // Servo power settings
    public static final double SHOOTER_SERVO_POWER = 1.0;
    
    // Trigger servo position settings (0.0 = 0 degrees, 1.0 = 180 degrees)
    public static final double TRIGGER_SERVO_MIN_POSITION = 0.222;    // 40 degrees (40/180 = 0.222)
    public static final double TRIGGER_SERVO_MAX_POSITION = 0.639;    // 115 degrees (115/180 = 0.639)
    
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
    
    // Shooter velocity control (ticks per second)
    public static double SHOOTER_TARGET_VELOCITY = 1600; // Range: 1200-1800 ticks/sec
    
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
        telemetry.addData("Instructions", "X = Indexor (120 degrees)");
        telemetry.addData("Instructions", "A = Intake + Converyor");
        telemetry.addData("Instructions", "Y = Shooter");
        telemetry.addData("Instructions", "B = Trigger Servo (40-115°) + AprilTag Align");
        telemetry.addData("Instructions", "Left Stick = Drive/Strafe, Right Stick X = Turn");
        telemetry.addData("AprilTag", "Looking for Blue ID %d", TARGET_TAG_ID);
        telemetry.update();
        
        waitForStart();
        
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            handleControllerInputs();
            handleMecanumDrive();
            handleAprilTagAlignment();
            checkIndexorCompletion();
            updateSpeedLight();
            updateTelemetry();
            sleep(20); // Small delay to prevent excessive CPU usage
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
        shooterServo.setDirection(DcMotorSimple.Direction.REVERSE);
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
        triggerServo.setPosition(TRIGGER_SERVO_MIN_POSITION); // Start at 40 degrees
        
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
        
        // Choose a camera resolution
        builder.setCameraResolution(new Size(640, 480));
        
        // Enable the RC preview (LiveView)
        builder.enableLiveView(true);
        
        // Set and enable the processor
        builder.addProcessor(aprilTag);
        
        // Build the Vision Portal
        visionPortal = builder.build();
        
        telemetry.addData("AprilTag Vision", "Initialized - Looking for Blue Tag ID %d", TARGET_TAG_ID);
        telemetry.update();
    }
    
    private void handleAprilTagAlignment() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        
        // Reset alignment status
        isAlignedToTag = false;
        
        // Look for the target tag
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == TARGET_TAG_ID) {
                // Found the target blue tag!
                double xOffset = detection.ftcPose.x;
                double yOffset = detection.ftcPose.y;
                double headingError = detection.ftcPose.yaw;
                
                // Check if we're aligned within tolerance
                boolean xAligned = Math.abs(xOffset) < ALIGNMENT_TOLERANCE;
                boolean yAligned = Math.abs(yOffset) < ALIGNMENT_TOLERANCE;
                boolean headingAligned = Math.abs(headingError) < HEADING_TOLERANCE;
                
                isAlignedToTag = xAligned && yAligned && headingAligned;
                
                // Provide alignment feedback
                telemetry.addData("AprilTag", "FOUND Blue Tag ID %d", TARGET_TAG_ID);
                telemetry.addData("Position", "X: %.1f, Y: %.1f", xOffset, yOffset);
                telemetry.addData("Heading Error", "%.1f degrees", headingError);
                telemetry.addData("Alignment Status", 
                    "X: %s, Y: %s, Heading: %s", 
                    xAligned ? "✓" : "✗", 
                    yAligned ? "✓" : "✗", 
                    headingAligned ? "✓" : "✗");
                
                if (isAlignedToTag) {
                    telemetry.addData("READY TO FIRE", "Trigger servo enabled!");
                } else {
                    telemetry.addData("Status", "Adjusting alignment...");
                    // Optional: Add automatic alignment control here
                    // You could use the offset values to automatically adjust robot position
                }
                
                return; // Found our tag, no need to check others
            }
        }
        
        // If we get here, the target tag wasn't found
        telemetry.addData("AprilTag", "Searching for Blue Tag ID %d...", TARGET_TAG_ID);
        telemetry.addData("Tags Detected", "%d total", currentDetections.size());
        
        // List any tags we can see
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addData("Detected Tag", "ID %d at distance %.1f inches", 
                    detection.id, detection.ftcPose.range);
            }
        }
        
        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        
        // Create the vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
    
    private void handleControllerInputs() {
        // Get current button states
        boolean currentX = gamepad1.x;
        boolean currentA = gamepad1.a;
        boolean currentY = gamepad1.y;
        boolean currentB = gamepad1.b;
        
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
        
        // Handle X button - Run Indexor for 3500 ticks
        if (currentX && !previousX) {
            runIndexorToPosition(INDEXOR_TICKS);
            //toggleIntakeAndConveryor();
        }
        
        // Handle A button - Toggle Intake and Converyor
        if (currentA && !previousA) {
            toggleIntakeAndConveryor();
        }
        
        // Handle Y button - Toggle Shooter
        if (currentY && !previousY) {
            toggleShooter();
        }
        
        // Handle B button - Toggle Trigger Servo
        if (currentB && !previousB) {
            toggleTriggerServo();
        }
        
        // Update previous button states
        previousX = currentX;
        previousA = currentA;
        previousY = currentY;
        previousB = currentB;
    }
    
    private void handleMecanumDrive() {
        // Get joystick inputs
        double drive = -gamepad1.left_stick_y;  // Forward/backward (negative because y-axis is inverted)
        double strafe = gamepad1.left_stick_x;  // Left/right strafe
        double turn = gamepad1.right_stick_x;   // Rotation
        
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
        indexor.setPower(0.8);
        
        // Start conveyor to work with indexor
        conveyor.setPower(CONVEYOR_POWER);
        
        telemetry.addData("Indexor", "Moving to position: %d", targetPosition);
        telemetry.addData("Converyor", "RUNNING with indexor");
        telemetry.update();
    }
    
    private void checkIndexorCompletion() {
        // Check if indexor was running in position mode and has completed
        if (indexor.getMode() == DcMotor.RunMode.RUN_TO_POSITION && !indexor.isBusy()) {
            // Indexor has reached target, stop conveyor if it was started by indexor
            // Only stop if intake is not running (A button control)
            if (Math.abs(intake.getPower()) < 0.1) {
                conveyor.setPower(0);
                telemetry.addData("Converyor", "STOPPED - Indexor completed");
            }
        }
    }
    
    private void toggleIntakeAndConveryor() {
        // Check if either motor is running
        boolean isRunning = (Math.abs(intake.getPower()) > 0.1) || 
                           (Math.abs(conveyor.getPower()) > 0.1);
        
        if (isRunning) {
            // Stop both motors
            intake.setPower(0);
            conveyor.setPower(0);
            telemetry.addData("Intake & Converyor", "STOPPED");
        } else {
            // Start both motors
            intake.setPower(INTAKE_POWER);
            conveyor.setPower(CONVEYOR_POWER);
            telemetry.addData("Intake & Converyor", "RUNNING");
        }
        telemetry.update();
    }
    
    private void toggleShooter() {
        // Check if shooter is running
        boolean isRunning = Math.abs(shooter.getVelocity()) > 50; // Check velocity instead of power
        
        if (isRunning) {
            // Stop shooter and shooter servo
            shooter.setVelocity(0);
            shooterServo.setPower(0);
            // Turn off speed light when shooter stops
            speedLight.setPosition(LIGHT_OFF_POSITION);
            telemetry.addData("Shooter", "STOPPED");
            telemetry.addData("Shooter Servo", "STOPPED");
            telemetry.addData("Speed Light", "OFF");
        } else {
            // Start shooter with target velocity and shooter servo
            shooter.setVelocity(SHOOTER_TARGET_VELOCITY);
            shooterServo.setPower(SHOOTER_SERVO_POWER);
            telemetry.addData("Shooter", "RUNNING at %.0f ticks/sec", SHOOTER_TARGET_VELOCITY);
            telemetry.addData("Shooter Servo", "RUNNING at %.2f power", SHOOTER_SERVO_POWER);
            telemetry.addData("Speed Light", "Monitoring speed...");
        }
        telemetry.update();
    }
    
    private void toggleTriggerServo() {
        // First check for AprilTag alignment before triggering
        if (!isAlignedToTag) {
            telemetry.addData("Trigger Servo", "Aligning to AprilTag first...");
            telemetry.addData("Status", "Looking for Blue Tag ID %d", TARGET_TAG_ID);
            telemetry.update();
            return; // Don't trigger until aligned
        }
        
        // Check current position and toggle between 40 and 115 degrees
        double currentPosition = triggerServo.getPosition();
        
        if (Math.abs(currentPosition - TRIGGER_SERVO_MIN_POSITION) < 0.1) {
            // Currently at 40 degrees, move to 115 degrees
            triggerServo.setPosition(TRIGGER_SERVO_MAX_POSITION);
            telemetry.addData("Trigger Servo", "FIRING! Moving to 115 degrees");
            telemetry.addData("AprilTag", "Aligned and fired!");
        } else {
            // Currently at 115 degrees (or somewhere else), move to 40 degrees
            triggerServo.setPosition(TRIGGER_SERVO_MIN_POSITION);
            telemetry.addData("Trigger Servo", "Resetting to 40 degrees");
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
    
    private void updateTelemetry() {
        // Display motor status
        telemetry.addData("Motor Status", "");
        telemetry.addData("Indexor Position", indexor.getCurrentPosition());
        telemetry.addData("Indexor Power", "%.2f", indexor.getPower());
        telemetry.addData("Intake Power", "%.2f", intake.getPower());
        telemetry.addData("Shooter Velocity", "%.0f / %.0f ticks/sec", 
                         shooter.getVelocity(), SHOOTER_TARGET_VELOCITY);
        telemetry.addData("Shooter Power", "%.2f", shooter.getPower());
        telemetry.addData("Converyor Power", "%.2f", conveyor.getPower());
        telemetry.addData("Shooter Servo Power", "%.2f", shooterServo.getPower());
        telemetry.addData("Trigger Servo Position", "%.3f (%.0f°)", 
                         triggerServo.getPosition(), triggerServo.getPosition() * 180);
        
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
        telemetry.addData("Controls", "");
        telemetry.addData("X Button", "Move Indexor 120 degrees");
        telemetry.addData("A Button", "Toggle Intake + Converyor");
        telemetry.addData("Y Button", "Toggle Shooter + Shooter Servo");
        telemetry.addData("B Button", "Toggle Trigger Servo (40-115°)");
        telemetry.addData("Left Stick", "Drive Forward/Back & Strafe Left/Right");
        telemetry.addData("Right Stick X", "Turn Left/Right");
        telemetry.addData("DPad Up", "Manual Green Light Test");
        telemetry.addData("DPad Down", "Manual Light Off Test");
        telemetry.addData("DPad Left", "Manual White Light Test");
        telemetry.addData("DPad Right", "Manual Test Position");
        
        // Check if indexor is busy
        if (indexor.isBusy()) {
            telemetry.addData("Indexor", "Moving to target...");
        }
        
        telemetry.update();
    }
}