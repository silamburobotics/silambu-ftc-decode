package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@Config
@Autonomous(name = "AutoOpDECODESimple", group = "Autonomous")
public class AutoOpDECODESimple extends LinearOpMode {
    
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
    
    // Declare odometry sensor
    private GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    
    // Motor power settings
    public static final double INTAKE_POWER = 0.8;
    public static final double CONVEYOR_POWER = 1.0;
    public static final double AUTO_INDEXOR_POWER = 0.1;      // Power for automatic indexor movement
    public static final double SHOOTER_POWER = 1.0;
    public static final double SHOOTER_SERVO_POWER = 1.0;     // Positive for forward direction
    
    // Indexor position settings
    public static final int INDEXOR_TICKS = 179;              // goBILDA 312 RPM motor: 120 degrees = 179 ticks
    
    // Shooter velocity control (ticks per second)
    public static double SHOOTER_TARGET_VELOCITY = 1600;      // Range: 1200-1800 ticks/sec
    public static final double SHOOTER_SPEED_THRESHOLD = 0.95; // 95% of target speed
    public static final double SHOOTER_TICKS_PER_REVOLUTION = 1020.0; // goBILDA 435 RPM motor
    
    // Speed light control settings (using servo positions for LED control)
    public static final double LIGHT_OFF_POSITION = 0.0;      // Servo position for light off
    public static final double LIGHT_GREEN_POSITION = 0.5;    // Servo position for green light
    public static final double LIGHT_WHITE_POSITION = 1.0;    // Servo position for white light
    
    // Trigger servo positions
    public static final double TRIGGER_FIRE = 0.10;     // 27 degrees (27/180 = 0.15) - FIRE POSITION (compact)
    public static final double TRIGGER_HOME = 0.76;     // 137 degrees (137/180 = 0.76) - HOME/SAFE POSITION (retracted)
    
    // Autonomous timing settings
    public static final double TRIGGER_FIRE_DURATION = 0.5;   // Seconds to stay in fire position
    public static final double WAIT_BETWEEN_SHOTS = 1.0;      // Seconds to wait between shots
    public static final double INDEXOR_MOVE_TIMEOUT = 3.0;    // Maximum time to wait for indexor movement
    public static final double SHOOTER_SPINUP_TIMEOUT = 5.0;  // Maximum time to wait for shooter to reach speed
    
    // Movement settings
    public static final double DRIVE_POWER = 0.6;             // Power for autonomous movement
    public static final double MOVEMENT_DISTANCE_FEET = 2.0;  // Distance to move (2 feet)
    public static final double MOVEMENT_TIMEOUT = 5.0;        // Maximum time for movement
    public static final double VELOCITY_THRESHOLD = 1400.0;   // Velocity threshold for direction decision
    
    // Encoder settings for movement (adjust based on your robot's wheel configuration)
    public static final double TICKS_PER_INCH = 50.0;         // Approximate ticks per inch (adjust for your wheels)
    public static final double INCHES_PER_FOOT = 12.0;        // Inches in a foot
    
    // Robot Front Direction Definition
    // IMPORTANT: Robot front is defined as the side with the shooter mechanism
    // - Positive Y direction = Robot moves forward (shooter side leads)
    // - Positive X direction = Robot strafes right
    // - Positive heading (0°) = Robot faces forward along positive Y axis
    // - Heading increases counter-clockwise (standard mathematical convention)
    
    // Pinpoint Odometry settings
    public static final double PINPOINT_X_OFFSET = -84.0;     // mm, X offset of the odometry pod from center of rotation
    public static final double PINPOINT_Y_OFFSET = -168.0;    // mm, Y offset of the odometry pod from center of rotation
    public static final double PINPOINT_YAW_SCALAR = 1.0;     // Scalar to adjust yaw measurement
    public static final double PINPOINT_X_SCALAR = 1.0;       // Scalar to adjust X measurement  
    public static final double PINPOINT_Y_SCALAR = 1.0;       // Scalar to adjust Y measurement
    
    @Override
    public void runOpMode() {
        // Initialize motors
        initializeMotors();
        
        // Display autonomous sequence
        telemetry.addData("Status", "AutoOpDECODESimple - Initialized");
        telemetry.addData("=== AUTONOMOUS SEQUENCE ===", "");
        telemetry.addData("1.", "Start shooter + servo + conveyor");
        telemetry.addData("2.", "Wait for max velocity");
        telemetry.addData("3.", "Fire shot 1");
        telemetry.addData("4.", "Move indexor + wait for velocity");
        telemetry.addData("5.", "Fire shot 2");
        telemetry.addData("6.", "Move indexor + wait for velocity");
        telemetry.addData("7.", "Fire shot 3");
        telemetry.addData("8.", "Stop shooter system");
        telemetry.addData("9.", "Move robot based on velocity");
        telemetry.addData("", "");
        telemetry.addData("Shooter Speed", "%.0f ticks/sec", SHOOTER_TARGET_VELOCITY);
        telemetry.addData("Movement Direction", SHOOTER_TARGET_VELOCITY > VELOCITY_THRESHOLD ? "Forward 2ft" : "Backward 2ft");
        telemetry.addData("Conveyor Power", "%.0f%%", CONVEYOR_POWER * 100);
        telemetry.addData("Shots", "3 total shots");
        telemetry.addData("Total Time", "~15-20 seconds (with movement)");
        telemetry.addData("", "");
        telemetry.addData("Odometry Status", odo.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", "%.1f Hz", odo.getFrequency());
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            executeAutonomousSequence();
        }
    }
    
    private void executeAutonomousSequence() {
        telemetry.addData("🤖 AUTONOMOUS", "Starting sequence...");
        updateOdometry();
        telemetry.update();
        
        // Step 1: Start shooter and servo
        startShooterSystem();
        
        // Step 2: Wait for shooter to reach maximum velocity
        waitForShooterSpeed();
        
        // Step 3: Fire first shot
        fireShot(1);
        
        // Step 4: Move indexor and wait for shooter speed
        moveIndexorToNextPosition();
        waitForShooterSpeed();
        
        // Step 5: Fire second shot (with velocity check)
        fireShot(2);
        
        // Step 6: Move indexor and wait for shooter speed
        moveIndexorToNextPosition();
        waitForShooterSpeed();
        
        // Step 7: Fire third shot (with velocity check)
        fireShot(3);
        
        // Step 8: Stop shooter
        stopShooterSystem();
        
        // Step 9: Move robot based on shooter velocity setting
        moveRobotBasedOnVelocity();
        
        telemetry.addData("✅ AUTONOMOUS", "Sequence completed!");
        telemetry.addData("🎯 Shots Fired", "3 shots");
        telemetry.addData("🚗 Movement", "2 feet %s", SHOOTER_TARGET_VELOCITY > VELOCITY_THRESHOLD ? "forward" : "backward");
        telemetry.addData("⏱️ Status", "Autonomous finished");
        telemetry.update();
    }
    
    private void startShooterSystem() {
        telemetry.addData("🚀 STEP 1", "Starting shooter system...");
        telemetry.update();
        
        // Start shooter with velocity control
        shooter.setVelocity(SHOOTER_TARGET_VELOCITY);
        
        // Start shooter servo
        shooterServo.setPower(SHOOTER_SERVO_POWER);
        
        // Start conveyor to help feed balls
        conveyor.setPower(CONVEYOR_POWER);
        
        telemetry.addData("✅ Shooter", "Started at %.0f ticks/sec", SHOOTER_TARGET_VELOCITY);
        telemetry.addData("✅ Shooter Servo", "Running at %.1f power", SHOOTER_SERVO_POWER);
        telemetry.addData("✅ Conveyor", "Running at %.1f power", CONVEYOR_POWER);
        telemetry.update();
    }
    
    private void waitForShooterSpeed() {
        telemetry.addData("⏳ STEP 2", "Waiting for shooter to reach speed...");
        telemetry.update();
        
        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();
        
        while (opModeIsActive() && timeout.seconds() < SHOOTER_SPINUP_TIMEOUT) {
            double currentVelocity = shooter.getVelocity();
            double speedPercentage = currentVelocity / SHOOTER_TARGET_VELOCITY;
            
            // Update speed light
            updateSpeedLight(currentVelocity);
            
            telemetry.addData("🎯 Target Speed", "%.0f ticks/sec", SHOOTER_TARGET_VELOCITY);
            telemetry.addData("⚡ Current Speed", "%.0f ticks/sec (%.0f%%)", 
                currentVelocity, speedPercentage * 100);
            telemetry.addData("💡 Speed Light", getSpeedLightStatus(currentVelocity));
            telemetry.addData("⏱️ Elapsed", "%.1f / %.1f seconds", timeout.seconds(), SHOOTER_SPINUP_TIMEOUT);
            updateOdometry();
            
            // Check if we've reached target speed
            if (speedPercentage >= SHOOTER_SPEED_THRESHOLD) {
                telemetry.addData("✅ READY", "Shooter at optimal speed!");
                updateOdometry();
                telemetry.update();
                return;
            }
            
            telemetry.update();
            sleep(50);
        }
        
        // If we get here, we timed out
        telemetry.addData("⚠️ WARNING", "Shooter spinup timeout - proceeding anyway");
        telemetry.update();
    }
    
    private void fireShot(int shotNumber) {
        telemetry.addData("🎯 PREPARING", "Shot %d of 3", shotNumber);
        telemetry.update();
        
        // Check and ensure shooter velocity before each shot
        ensureShooterVelocity(shotNumber);
        
        telemetry.addData("🎯 FIRING", "Shot %d of 3 - Velocity Ready", shotNumber);
        telemetry.update();
        
        // Move trigger to fire position
        triggerServo.setPosition(TRIGGER_FIRE);
        telemetry.addData("🎯 Trigger", "FIRE position (%.0f°)", TRIGGER_FIRE * 180);
        telemetry.update();
        
        // Wait for fire duration
        sleep((long)(TRIGGER_FIRE_DURATION * 1000));
        
        // Return trigger to home position
        triggerServo.setPosition(TRIGGER_HOME);
        telemetry.addData("🏠 Trigger", "HOME position (%.0f°)", TRIGGER_HOME * 180);
        telemetry.addData("✅ Shot %d", "Fired successfully!", shotNumber);
        telemetry.update();
    }
    
    private void ensureShooterVelocity(int shotNumber) {
        double currentVelocity = shooter.getVelocity();
        double speedPercentage = currentVelocity / SHOOTER_TARGET_VELOCITY;
        
        telemetry.addData("🔍 VELOCITY CHECK", "Shot %d velocity status", shotNumber);
        telemetry.addData("⚡ Current Speed", "%.0f ticks/sec (%.0f%%)", 
            currentVelocity, speedPercentage * 100);
        telemetry.addData("🎯 Target Speed", "%.0f ticks/sec", SHOOTER_TARGET_VELOCITY);
        
        // If velocity is good, no need to wait
        if (speedPercentage >= SHOOTER_SPEED_THRESHOLD) {
            telemetry.addData("✅ VELOCITY", "Already at optimal speed!");
            telemetry.update();
            return;
        }
        
        // Velocity has dropped - wait for recovery
        telemetry.addData("⚠️ VELOCITY", "Below threshold - waiting for recovery...");
        telemetry.update();
        
        // Re-ensure shooter is running at target velocity
        shooter.setVelocity(SHOOTER_TARGET_VELOCITY);
        
        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();
        
        while (opModeIsActive() && timeout.seconds() < SHOOTER_SPINUP_TIMEOUT) {
            currentVelocity = shooter.getVelocity();
            speedPercentage = currentVelocity / SHOOTER_TARGET_VELOCITY;
            
            // Update speed light
            updateSpeedLight(currentVelocity);
            
            telemetry.addData("🔄 RECOVERY", "Shot %d - waiting for velocity", shotNumber);
            telemetry.addData("⚡ Current Speed", "%.0f ticks/sec (%.0f%%)", 
                currentVelocity, speedPercentage * 100);
            telemetry.addData("💡 Speed Light", getSpeedLightStatus(currentVelocity));
            telemetry.addData("⏱️ Recovery Time", "%.1f / %.1f seconds", timeout.seconds(), SHOOTER_SPINUP_TIMEOUT);
            
            // Check if we've reached target speed
            if (speedPercentage >= SHOOTER_SPEED_THRESHOLD) {
                telemetry.addData("✅ RECOVERED", "Velocity restored for shot %d!", shotNumber);
                telemetry.update();
                return;
            }
            
            telemetry.update();
            sleep(50);
        }
        
        // If we get here, we timed out
        telemetry.addData("⚠️ WARNING", "Shot %d velocity recovery timeout - firing anyway", shotNumber);
        telemetry.addData("⚡ Final Speed", "%.0f ticks/sec (%.0f%%)", 
            currentVelocity, speedPercentage * 100);
        telemetry.update();
    }
    
    private void moveIndexorToNextPosition() {
        telemetry.addData("🔄 INDEXOR", "Moving to next position...");
        telemetry.update();
        
        // Get current position and calculate target
        int currentPosition = indexor.getCurrentPosition();
        int targetPosition = currentPosition + INDEXOR_TICKS;
        
        // Set indexor to position mode
        indexor.setTargetPosition(targetPosition);
        indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexor.setPower(AUTO_INDEXOR_POWER);
        
        // Wait for movement to complete with timeout
        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();
        
        while (opModeIsActive() && indexor.isBusy() && timeout.seconds() < INDEXOR_MOVE_TIMEOUT) {
            int currentPos = indexor.getCurrentPosition();
            telemetry.addData("🎯 Target Position", "%d ticks", targetPosition);
            telemetry.addData("📍 Current Position", "%d ticks", currentPos);
            telemetry.addData("📏 Remaining", "%d ticks", Math.abs(targetPosition - currentPos));
            telemetry.addData("⏱️ Elapsed", "%.1f / %.1f seconds", timeout.seconds(), INDEXOR_MOVE_TIMEOUT);
            telemetry.update();
            sleep(50);
        }
        
        // Stop indexor
        indexor.setPower(0);
        
        if (timeout.seconds() >= INDEXOR_MOVE_TIMEOUT) {
            telemetry.addData("⚠️ WARNING", "Indexor movement timeout");
        } else {
            telemetry.addData("✅ INDEXOR", "Position reached");
        }
        telemetry.update();
    }
    
    private void stopShooterSystem() {
        telemetry.addData("🛑 STEP 8", "Stopping shooter system...");
        telemetry.update();
        
        // Stop shooter
        shooter.setPower(0);
        
        // Stop shooter servo
        shooterServo.setPower(0);
        
        // Stop conveyor
        conveyor.setPower(0);
        
        // Turn off speed light
        speedLight.setPosition(LIGHT_OFF_POSITION);
        
        telemetry.addData("✅ Shooter", "Stopped");
        telemetry.addData("✅ Shooter Servo", "Stopped");
        telemetry.addData("✅ Conveyor", "Stopped");
        telemetry.addData("💡 Speed Light", "Off");
        telemetry.update();
    }
    
    private void updateSpeedLight(double currentVelocity) {
        double speedPercentage = currentVelocity / SHOOTER_TARGET_VELOCITY;
        
        if (currentVelocity > 50 && speedPercentage >= SHOOTER_SPEED_THRESHOLD) {
            // Speed is good - green light
            speedLight.setPosition(LIGHT_GREEN_POSITION);
        } else if (currentVelocity > 50 && speedPercentage > 0.7) {
            // Speed is medium - white light
            speedLight.setPosition(LIGHT_WHITE_POSITION);
        } else {
            // Speed is low - off
            speedLight.setPosition(LIGHT_OFF_POSITION);
        }
    }
    
    private String getSpeedLightStatus(double currentVelocity) {
        double speedPercentage = currentVelocity / SHOOTER_TARGET_VELOCITY;
        
        if (currentVelocity > 50 && speedPercentage >= SHOOTER_SPEED_THRESHOLD) {
            return "🟢 GREEN (Ready)";
        } else if (currentVelocity > 50 && speedPercentage > 0.7) {
            return "⚪ WHITE (Medium)";
        } else {
            return "⚫ OFF (Low)";
        }
    }
    
    private void updateOdometry() {
        // Update the position of the robot
        odo.update();
        
        // Get the current position
        Pose2D pos = odo.getPosition();
        String data = String.format("X: %.1f, Y: %.1f, H: %.1f° (0° = Forward)", 
            pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("📍 Position", data);
        telemetry.addData("Odometry Status", odo.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", "%.1f Hz", odo.getFrequency());
    }
    
    private void moveRobotBasedOnVelocity() {
        // Determine direction based on shooter velocity
        boolean moveForward = SHOOTER_TARGET_VELOCITY > VELOCITY_THRESHOLD;
        String direction = moveForward ? "FORWARD" : "BACKWARD";
        
        telemetry.addData("🚗 STEP 9", "Moving robot %s...", direction);
        telemetry.addData("⚡ Velocity Setting", "%.0f ticks/sec", SHOOTER_TARGET_VELOCITY);
        telemetry.addData("🎯 Threshold", "%.0f ticks/sec", VELOCITY_THRESHOLD);
        telemetry.addData("📏 Distance", "%.1f feet", MOVEMENT_DISTANCE_FEET);
        updateOdometry();
        telemetry.update();
        
        // Calculate target distance in encoder ticks
        double targetDistanceInches = MOVEMENT_DISTANCE_FEET * INCHES_PER_FOOT;
        int targetTicks = (int)(targetDistanceInches * TICKS_PER_INCH);
        
        // Get starting position from odometry for reference
        odo.update();
        Pose2D startPose = odo.getPosition();
        double startY = startPose.getY(DistanceUnit.MM);
        
        // Reset and configure drive motor encoders for movement
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Set target positions (positive for forward, negative for backward)
        int targetPosition = moveForward ? targetTicks : -targetTicks;
        
        leftFront.setTargetPosition(targetPosition);
        rightFront.setTargetPosition(targetPosition);
        leftBack.setTargetPosition(targetPosition);
        rightBack.setTargetPosition(targetPosition);
        
        // Set to RUN_TO_POSITION mode
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // Start movement with appropriate power
        double movePower = moveForward ? DRIVE_POWER : -DRIVE_POWER;
        leftFront.setPower(movePower);
        rightFront.setPower(movePower);
        leftBack.setPower(movePower);
        rightBack.setPower(movePower);
        
        // Monitor movement progress
        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();
        
        while (opModeIsActive() && 
               (leftFront.isBusy() || rightFront.isBusy() || leftBack.isBusy() || rightBack.isBusy()) && 
               timeout.seconds() < MOVEMENT_TIMEOUT) {
            
            // Get current positions
            int currentLF = leftFront.getCurrentPosition();
            int currentRF = rightFront.getCurrentPosition();
            int currentLB = leftBack.getCurrentPosition();
            int currentRB = rightBack.getCurrentPosition();
            int avgPosition = (currentLF + currentRF + currentLB + currentRB) / 4;
            
            // Update odometry and get current position
            updateOdometry();
            odo.update();
            Pose2D currentPose = odo.getPosition();
            double currentY = currentPose.getY(DistanceUnit.MM);
            double distanceTraveledMM = Math.abs(currentY - startY);
            double distanceTraveledInches = distanceTraveledMM / 25.4; // Convert mm to inches
            
            telemetry.addData("🚗 MOVING", "%s %.1f feet", direction, MOVEMENT_DISTANCE_FEET);
            telemetry.addData("🎯 Target Position", "%d ticks", targetPosition);
            telemetry.addData("📍 Average Position", "%d ticks", avgPosition);
            telemetry.addData("📏 Progress", "%.1f / %.1f inches", distanceTraveledInches, targetDistanceInches);
            telemetry.addData("⏱️ Elapsed", "%.1f / %.1f seconds", timeout.seconds(), MOVEMENT_TIMEOUT);
            telemetry.addData("Motor Positions", "LF:%d RF:%d LB:%d RB:%d", currentLF, currentRF, currentLB, currentRB);
            telemetry.update();
            
            sleep(50);
        }
        
        // Stop all drive motors
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        
        // Get final position for verification
        odo.update();
        Pose2D finalPose = odo.getPosition();
        double finalY = finalPose.getY(DistanceUnit.MM);
        double totalDistanceMM = Math.abs(finalY - startY);
        double totalDistanceInches = totalDistanceMM / 25.4;
        double totalDistanceFeet = totalDistanceInches / 12.0;
        
        // Reset to run without encoders for safety
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        if (timeout.seconds() >= MOVEMENT_TIMEOUT) {
            telemetry.addData("⚠️ WARNING", "Movement timeout");
        } else {
            telemetry.addData("✅ MOVEMENT", "Completed %s", direction);
        }
        
        telemetry.addData("📏 Actual Distance", "%.2f feet (%.1f inches)", totalDistanceFeet, totalDistanceInches);
        telemetry.addData("🎯 Target Distance", "%.1f feet", MOVEMENT_DISTANCE_FEET);
        updateOdometry();
        telemetry.update();
        
        sleep(1000); // Brief pause to show results
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
        
        // Initialize Pinpoint Odometry Computer
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        
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
        // Motor directions are configured for robot front = shooter side
        // This setup ensures positive Y movement = forward (toward target)
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
        
        // Set mecanum drive motors to run without encoders
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set indexor to use encoder
        indexor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Configure Pinpoint Odometry Computer
        odo.setOffsets(PINPOINT_X_OFFSET, PINPOINT_Y_OFFSET, DistanceUnit.MM); // Set the odometry pod location relative to the point that the odometry computer tracks around
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD); // Set the encoder resolution
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD); // Set the direction that each encoder counts as positive
        odo.resetPosAndIMU(); // Reset the position to 0 and reset the IMU
    }
}