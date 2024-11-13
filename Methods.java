package org.firstinspires.ftc.teamcode;

// Imports all necessary packages
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Initializes the class
public class Methods {
    
    // Establishes the "robot" object from the "RobotMap" class
    RobotMap robot = new RobotMap();
    
    // Establishes the "runtime" object from the "ElapsedTime" class
    private ElapsedTime runtime = new ElapsedTime();
    
    private DcMotorEx leftFrontDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightBackDrive;
    private DcMotorEx extender;
    private Servo servo1;
    //private Servo servo2;
    
    public Methods (RobotMap robot) {
        leftFrontDrive = robot.leftFrontDrive;
        rightFrontDrive = robot.rightFrontDrive;
        leftBackDrive = robot.leftBackDrive;
        rightBackDrive = robot.rightBackDrive;
        extender = robot.extender;
        //servo1 = robot.servo1;
        //servo2 = robot.servo2;
    }
    
    public void turn(int degrees, int speed) {
        
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        
        // tell the robot where we are driving to
        // target position is in Ticks
        double targetPosition = (degrees * 2.54) * (537.7 / 114);
        if (degrees != 0) {
            leftBackDrive.setTargetPosition(-(int)(targetPosition)); 
            rightBackDrive.setTargetPosition((int)(targetPosition)); 
            rightFrontDrive.setTargetPosition((int)(targetPosition));
            leftFrontDrive.setTargetPosition(-(int)(targetPosition));
        }
        else {
            leftBackDrive.setTargetPosition(0);
            rightBackDrive.setTargetPosition(0);
            rightFrontDrive.setTargetPosition(0);
            leftFrontDrive.setTargetPosition(0);
        }
        
        // tell the robot to drive to where we need to
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        
        // use getCurrentPosition method to keep robot at the velocity while it isn't at the location
        while (Math.abs(leftFrontDrive.getCurrentPosition() - leftFrontDrive.getTargetPosition()) >= 5){
        // Sets speed of wheels
            leftFrontDrive.setVelocity(speed);
            rightFrontDrive.setVelocity(speed);
            leftBackDrive.setVelocity(speed);
            rightBackDrive.setVelocity(speed);
            
        }
        // set velocity to 0 when it reaches location
        leftFrontDrive.setVelocity(0);
        rightFrontDrive.setVelocity(0);
        leftBackDrive.setVelocity(0);
        rightBackDrive.setVelocity(0);
    }
    
    // method that will be used to move the robot however much we need
    // Distance is in inches
    public void drive(int distance, int speed) {
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        
        // tell the robot where we are driving to
        // target position is in Ticks
        double targetPosition = (distance * 2.54) * (537.7 / 30.69);
        leftBackDrive.setTargetPosition((int)(-targetPosition)); 
        rightBackDrive.setTargetPosition((int)(-targetPosition)); 
        rightFrontDrive.setTargetPosition((int)(-targetPosition));
        leftFrontDrive.setTargetPosition((int)(-targetPosition));
        
        // tell the robot to drive to where we need to
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        
        // use getCurrentPosition method to keep robot at the velocity while it isn't at the location
        while (Math.abs(leftFrontDrive.getCurrentPosition() - leftFrontDrive.getTargetPosition()) >= 5){
        // Sets speed of wheels
            leftFrontDrive.setVelocity(speed);
            rightFrontDrive.setVelocity(speed);
            leftBackDrive.setVelocity(speed);
            rightBackDrive.setVelocity(speed);

        }
        // set velocity to 0 when it reaches location
        // leftFrontDrive.setVelocity(0);
        // rightFrontDrive.setVelocity(0);
        // leftBackDrive.setVelocity(0);
        // rightBackDrive.setVelocity(0);
    }
        
        
    // Tatget position values: 0 = floor, 1 = ground, 2 = low, 3 = med, 4 = high
    // Spool diameter: 12 cm or 4.72 inches
    // The thingys are at 1.5, 13, 23, and 33 inches high
    //public void extend(int targetPosition) {
        // set the extender to use encoders
        //extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // Set the target position depending on the method input
        //switch(targetPosition) {
    //         case 0: // Floor
    //             //extender.setTargetPosition(0);
    //             break;
    //         case 1: // Ground terminal
    //             //extender.setTargetPosition(171);
    //             break;
    //         case 2: // Low terminal
    //             //extender.setTargetPosition(1230);
    //             break;
    //         case 3: // Medium terminal
    //             //extender.setTargetPosition(2117);
    //             break;
    //         case 4: // High terminal
    //             //extender.setTargetPosition(2906);
    //             break;
    //     }
        
    //     //while(Math.abs(extender.getTargetPosition() - extender.getCurrentPosition()) >= 5) {
    //         //extender.setVelocity(2000);
    //     }
        
    // }
    
    public void extend_precise(int targetPosition) {
        // set the extender to use encoders
        
        
        extender.setTargetPosition(extender.getCurrentPosition() + targetPosition);
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Math.abs(extender.getTargetPosition() - extender.getCurrentPosition()) >= 5) {
            extender.setVelocity(2000);
        }
    }
    
    // public void claw(boolean open) {
    //     if (open) {
    //         //servo1.setPosition(0.25);
    //         //servo2.setPosition(0.3);
    //     } else {
    //         //servo1.setPosition(0.05);
    //         //servo2.setPosition(0.5);
    //     }
    // }
    
    public void strafe(int distance, int speed) { //negative is left, positive is right
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        
        // tell the robot where we are driving to
        // target position is in Ticks
        double targetPosition = (Math.abs(distance) * 2.54) * (537.7 / 28.13);
        if (distance > 0) {
            leftBackDrive.setTargetPosition((int)(targetPosition)); 
            rightBackDrive.setTargetPosition((int)(-targetPosition)); 
            rightFrontDrive.setTargetPosition((int)(targetPosition));
            leftFrontDrive.setTargetPosition((int)(-targetPosition));
        }
        else if (distance < 0) {
            leftBackDrive.setTargetPosition((int)(-targetPosition)); 
            rightBackDrive.setTargetPosition((int)(targetPosition)); 
            rightFrontDrive.setTargetPosition((int)(-targetPosition));
            leftFrontDrive.setTargetPosition((int)(targetPosition));
        }
        else {
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
        }
        
        // tell the robot to drive to where we need to
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        
        // use getCurrentPosition method to keep robot at the velocity while it isn't at the location
        while (Math.abs(leftFrontDrive.getCurrentPosition() - leftFrontDrive.getTargetPosition()) >= 5){
        // Sets speed of wheels
            leftFrontDrive.setVelocity(speed);
            rightFrontDrive.setVelocity(speed);
            leftBackDrive.setVelocity(speed);
            rightBackDrive.setVelocity(speed);
        }
        // set velocity to 0 when it reaches location
        leftFrontDrive.setVelocity(0);
        rightFrontDrive.setVelocity(0);
        leftBackDrive.setVelocity(0);
        rightBackDrive.setVelocity(0);
    }
}

