package org.firstinspires.ftc.teamcode;

// Imports all necessary packages
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.*;


// Initializes the class
@TeleOp(name="Tank Mode")
public class TankMode extends OpMode {
    int armSpeed = 2000;
    
    
    
    static final double FEET_PER_METER = 3.28084;
    /// Establishes the "robot" object from the "RobotMap" class
    RobotMap robot = new RobotMap();
    
    // Establishes the "runtime" object from the "ElapsedTime" class
    private ElapsedTime runtime = new ElapsedTime();
    
    // Code that runs when the driver hits INIT
    @Override
    public void init() {
        // Initialize the hardware variables
        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Send telemetry message to signify robot waiting
        telemetry.addData("Last Modified: October 4th, 2022", "Hello MATES Drivers!");
    }
    
    //Code that runs repeatedly after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
        // Ensures the power is off on all the motors and the servos are closed
        robot.leftFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        robot.extender.setPower(0);
        robot.servo1.setPosition(0.3);
        robot.servo2.setPosition(0.2);
        // robot.servo1.setPosition(0);
        // robot.servo2.setPosition(0.5);
    }
    
    // Code that runs when the driver hits PLAY
    @Override
    public void start() {
        // Ensures the power is off on all the motors and the servos are closed
        robot.leftFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        robot.extender.setPower(0);
        robot.servo1.setPosition(0.3);
        robot.servo2.setPosition(0.23);
        // robot.servo1.setPosition(0);
        // robot.servo2.setPosition(0.5);
    }
    
    // Ensures the up and down button inputs happen only once when pushed
    boolean buttonPress = false;
    boolean buttonPressDown = false;
    boolean buttonPressUp = false;
    
    // speedSelected: 4 = low, 2 = med, 1 = high
    int speedSelected = 2; // temp
    
    // Claw position and state. pos: 0=floor, 1=ground, 2=low, 3=medium, 4=high
    int clawPosition = 0;
    boolean clawOpen = false;
    
    // Code that runs repeatedly after the driver hits PLAY, but before they hit STOP
    @Override
    public void loop() {
        //if y is pressed on controller, it selects the high speed setting
        if (gamepad1.y) { 
            speedSelected = 1;
        }
        //if b is pressed on controller, it selects the h speed setting
        else if (gamepad1.b) {
            speedSelected = 2;
        }
        //if a is pressed on controller, it selects the low speed setting
        else if (gamepad1.a) {
            speedSelected = 4;
        }
        
        // Left stick and right stick make robot drive
        //speed is changed depending on button pressed, divides to reduce speed
        else if (Math.abs(gamepad1.left_stick_y) > 0.2 || Math.abs(gamepad1.right_stick_y) > 0.2) {
            robot.leftFrontDrive.setPower(gamepad1.left_stick_y / speedSelected);
            robot.leftBackDrive.setPower(gamepad1.left_stick_y / speedSelected);
            robot.rightFrontDrive.setPower(gamepad1.right_stick_y / speedSelected);
            robot.rightBackDrive.setPower(gamepad1.right_stick_y / speedSelected);
        }
        
        else if ((Math.abs(gamepad2.left_stick_y) > 0.2 || Math.abs(gamepad2.right_stick_y) > 0.2)) {
            robot.leftFrontDrive.setPower(gamepad2.left_stick_y / speedSelected);
            robot.leftBackDrive.setPower(gamepad2.left_stick_y / speedSelected);
            robot.rightFrontDrive.setPower(gamepad2.right_stick_y / speedSelected);
            robot.rightBackDrive.setPower(gamepad2.right_stick_y / speedSelected);
        }
        
        // Left bumper makes robot strafe left
        //we are yet to figure out why the *1.5 works, but it does
        else if (gamepad1.left_bumper || gamepad2.left_bumper) {
            robot.leftFrontDrive.setPower(1 / (speedSelected * 1.5));
            robot.leftBackDrive.setPower(-1 / (speedSelected * 1.5));
            robot.rightFrontDrive.setPower(-1 / (speedSelected * 1.5));
            robot.rightBackDrive.setPower(1 / (speedSelected * 1.5));
        }
        
        // Right bumper makes robot strafe right
        else if (gamepad1.right_bumper || gamepad2.right_bumper) {
            robot.leftFrontDrive.setPower(-1 / (speedSelected * 1.5));
            robot.leftBackDrive.setPower(1 / (speedSelected * 1.5));
            robot.rightFrontDrive.setPower(1 / (speedSelected * 1.5));
            robot.rightBackDrive.setPower(-1 / (speedSelected * 1.5));
        }
        
        // // D-pad up raises arm
        // else if ((gamepad1.dpad_up || gamepad2.dpad_up) 
        //     && buttonPress == false && clawPosition < 4) {
        //     clawPosition++;
        //     if (clawPosition == 1) {
        //         robot.extender.setTargetPosition(171);
        //         robot.extender.setVelocity(armSpeed);
        //         buttonPress = true;
        //     }
        //     else if (clawPosition == 2) {
        //         robot.extender.setTargetPosition(1230);
        //         robot.extender.setVelocity(armSpeed);
        //         buttonPress = true;
        //     }
        //     else if (clawPosition == 3) {
        //         robot.extender.setTargetPosition(2117);
        //         robot.extender.setVelocity(armSpeed);
        //         buttonPress = true;
        //     }
        //     else if (clawPosition == 4) {
        //         robot.extender.setTargetPosition(2906);
        //         robot.extender.setVelocity(armSpeed);
        //         buttonPress = true;
        //     }
        // }
        
        // // D-pad down lowers arm
        // else if ((gamepad1.dpad_down || gamepad2.dpad_down) &&
        //     buttonPress == false && clawPosition > 0) {
        //     clawPosition--;
        //     if (clawPosition == 0) {
        //         robot.extender.setTargetPosition(15);
        //         robot.extender.setVelocity(armSpeed);
        //         buttonPress = true;
        //     }
        //     else if (clawPosition == 1) {
        //         robot.extender.setTargetPosition(171);
        //         robot.extender.setVelocity(armSpeed);
        //         buttonPress = true;
        //     }
        //     else if (clawPosition == 2) {
        //         robot.extender.setTargetPosition(1230);
        //         robot.extender.setVelocity(armSpeed);
        //         buttonPress = true;
        //     }
        //     else if (clawPosition == 3) {
        //         robot.extender.setTargetPosition(2117);
        //         robot.extender.setVelocity(armSpeed);
        //         buttonPress = true;
        //     }
            
        // }
        
        // Right trigger manually adjusts the claw up
        else if (gamepad1.right_trigger > 0.5) {
            
            robot.extender.setPower(-0.1);
            buttonPressDown = true;
            buttonPressUp = false;
        }
        
        
        // Left trigger manually adjusts the claw down
        else if (gamepad1.left_trigger > 0.5 ) {
            
            robot.extender.setPower(-0.5);
            buttonPressUp = true;
            buttonPressDown = false;
        }
        
        else if (gamepad1.dpad_up || gamepad2.dpad_up || gamepad1.dpad_down || gamepad2.dpad_down) {
            
        }
        
        // D-pad left opens claw
        else if (gamepad1.dpad_left || gamepad2.dpad_left) {
            // robot.servo1.setPosition(0.2);
            robot.servo1.setPosition(1);
        }
        
        // D-pad right closes claw
        else if (gamepad1.dpad_right || gamepad2.dpad_right) {
            //robot.servo1.setPosition(0.0);
            //robot.servo2.setPosition(0.5);
            robot.servo1.setPosition(0.05);
        }
        
        // Keeps motors off when nothing is pressed
        else {
            buttonPress = false;
            robot.leftFrontDrive.setPower(0);
            robot.leftBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            if (buttonPressUp){
            robot.extender.setPower(-0.4);
            }
            else{
                robot.extender.setPower(0);
            }
        }
        telemetry.addData("Current height: " + clawPosition, "Current ticks: " + robot.extender.getCurrentPosition());
    }
    
    // Code that runs once the driver hits STOP
    @Override
    public void stop() {
        // Ensures the power is off on all the motors and the servos are closed
        robot.extender.setTargetPosition(0);
        robot.extender.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.extender.setVelocity(0);
        robot.leftFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        robot.servo1.setPosition(0);
        robot.servo2.setPosition(0.5);
    }
    
}