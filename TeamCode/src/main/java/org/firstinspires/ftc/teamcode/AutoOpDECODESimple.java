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
    
    @Override
    public void runOpMode() {
        // Initialize motors
        initializeMotors();
        
        // Display autonomous sequence
        telemetry.addData("Status", "AutoOpDECODESimple - Initialized");
        telemetry.addData("=== AUTONOMOUS SEQUENCE ===", "");
        telemetry.addData("1.", "Start shooter + servo + conveyor");
        telemetry.addData("2.", "Wait for max velocity");
        telemetry.addData("3.", "Fire shots 1 & 2 (position 1)");
        telemetry.addData("4.", "Move indexor to position 2");
        telemetry.addData("5.", "Fire shots 3 & 4 (position 2)");
        telemetry.addData("6.", "Move indexor to position 3");
        telemetry.addData("7.", "Fire shots 5 & 6 (position 3)");
        telemetry.addData("8.", "Stop shooter system");
        telemetry.addData("", "");
        telemetry.addData("Shooter Speed", "%.0f ticks/sec", SHOOTER_TARGET_VELOCITY);
        telemetry.addData("Conveyor Power", "%.0f%%", CONVEYOR_POWER * 100);
        telemetry.addData("Shots", "6 total shots (2 per position)");
        telemetry.addData("Total Time", "~25-30 seconds");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            executeAutonomousSequence();
        }
    }
    
    private void executeAutonomousSequence() {
        telemetry.addData("🤖 AUTONOMOUS", "Starting sequence...");
        telemetry.update();
        
        // Step 1: Start shooter and servo
        startShooterSystem();
        
        // Step 2: Wait for shooter to reach maximum velocity
        waitForShooterSpeed();
        
        // Step 3: Fire first two shots at initial position
        fireShot(1);
        sleep((long)(WAIT_BETWEEN_SHOTS * 1000));
        fireShot(2);
        
        // Step 4: Wait, move indexor, wait
        sleep((long)(WAIT_BETWEEN_SHOTS * 1000));
        moveIndexorToNextPosition();
        sleep((long)(WAIT_BETWEEN_SHOTS * 1000));
        
        // Step 5: Fire next two shots at second position
        fireShot(3);
        sleep((long)(WAIT_BETWEEN_SHOTS * 1000));
        fireShot(4);
        
        // Step 6: Wait, move indexor, wait
        sleep((long)(WAIT_BETWEEN_SHOTS * 1000));
        moveIndexorToNextPosition();
        sleep((long)(WAIT_BETWEEN_SHOTS * 1000));
        
        // Step 7: Fire final two shots at third position
        fireShot(5);
        sleep((long)(WAIT_BETWEEN_SHOTS * 1000));
        fireShot(6);
        
        // Step 8: Wait and stop shooter
        sleep((long)(WAIT_BETWEEN_SHOTS * 1000));
        stopShooterSystem();
        
        telemetry.addData("✅ AUTONOMOUS", "Sequence completed!");
        telemetry.addData("🎯 Shots Fired", "6 shots total");
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
            
            // Check if we've reached target speed
            if (speedPercentage >= SHOOTER_SPEED_THRESHOLD) {
                telemetry.addData("✅ READY", "Shooter at optimal speed!");
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
        telemetry.addData("🎯 FIRING", "Shot %d of 3", shotNumber);
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
        
        // Set mecanum drive motors to run without encoders
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set indexor to use encoder
        indexor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}