package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
@Config
@TeleOp(name = "Four Motor TeleOp", group = "TeleOp")
public class FourMotorTeleOp extends LinearOpMode {
    
    // Declare motors
    private DcMotorEx indexor;
    private DcMotorEx intake;
    private DcMotorEx shooter;
    private DcMotorEx conveyor; // Fixed typo from "notor_converyor"
    
    // Declare servo
    private CRServo shooterServo;
    
    // Variables to track button states
    private boolean previousX = false;
    private boolean previousA = false;
    private boolean previousY = false;
    
    // Motor power settings
    public static final double INTAKE_POWER = 0.8;
    public static final double SHOOTER_POWER = 1.0;
    public static final double CONVEYOR_POWER = 1.0;
    public static  int INDEXOR_TICKS = 3800;
    
    // Servo power settings
    public static final double SHOOTER_SERVO_POWER = 1.0;
    
    // Shooter velocity control (ticks per second)
    public static double SHOOTER_TARGET_VELOCITY = 1600; // Range: 1200-1800 ticks/sec
    
    @Override
    public void runOpMode() {
        // Initialize motors
        initializeMotors();
        
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "X = Indexor (3500 ticks)");
        telemetry.addData("Instructions", "A = Intake + Converyor");
        telemetry.addData("Instructions", "Y = Shooter");
        telemetry.update();
        
        waitForStart();
        
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            handleControllerInputs();
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
        
        // Initialize servo
        shooterServo = hardwareMap.get(CRServo.class, "shooterServo");
        
        // Set motor directions (adjust as needed for your robot)
        indexor.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        conveyor.setDirection(DcMotor.Direction.REVERSE);
        
        // Set servo direction
        shooterServo.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Set zero power behavior
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize servo to stopped
        shooterServo.setPower(0);
        
        // Reset encoders
        indexor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set indexor to use encoder
        indexor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set shooter to use velocity control
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    private void handleControllerInputs() {
        // Get current button states
        boolean currentX = gamepad1.x;
        boolean currentA = gamepad1.a;
        boolean currentY = gamepad1.y;
        
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
        
        // Update previous button states
        previousX = currentX;
        previousA = currentA;
        previousY = currentY;
    }
    
    private void runIndexorToPosition(int ticks) {
        // Set target position
        int currentPosition = indexor.getCurrentPosition();
        int targetPosition = currentPosition + ticks;
        
        indexor.setTargetPosition(targetPosition);
        indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexor.setPower(0.8);
        
        telemetry.addData("Indexor", "Moving to position: %d", targetPosition);
        telemetry.update();
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
            // Stop shooter and continue servo
            shooter.setVelocity(0);
            shooterServo.setPower(0);
            telemetry.addData("Shooter", "STOPPED");
            telemetry.addData("Shooter Servo", "STOPPED");
        } else {
            // Start shooter with target velocity and continue servo
            shooter.setVelocity(SHOOTER_TARGET_VELOCITY);
            shooterServo.setPower(SHOOTER_SERVO_POWER);
            telemetry.addData("Shooter", "RUNNING at %.0f ticks/sec", SHOOTER_TARGET_VELOCITY);
            telemetry.addData("Shooter Servo", "RUNNING at %.2f power", SHOOTER_SERVO_POWER);
        }
        telemetry.update();
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
        
        // Display button instructions
        telemetry.addData("", "");
        telemetry.addData("Controls", "");
        telemetry.addData("X Button", "Move Indexor 3500 ticks");
        telemetry.addData("A Button", "Toggle Intake + Converyor");
        telemetry.addData("Y Button", "Toggle Shooter + Shooter Servo");
        
        // Check if indexor is busy
        if (indexor.isBusy()) {
            telemetry.addData("Indexor", "Moving to target...");
        }
        
        telemetry.update();
    }
}