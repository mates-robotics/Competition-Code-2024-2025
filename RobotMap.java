// GOBILDA Password: matesrobotics123

package org.firstinspires.ftc.teamcode;

// Imports all necessary packages
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Initializes the class
public class RobotMap {
    
    // Creates variables for all of the motors
    public DcMotorEx leftFrontDrive = null;
    public DcMotorEx rightFrontDrive = null;
    public DcMotorEx leftBackDrive = null;
    public DcMotorEx rightBackDrive = null;
    public DcMotorEx extender = null;
    public Servo servo1 = null;
    public Servo servo2 = null;
    
    // Establishes the "runtime" object from the "ElapsedTime" class
    private ElapsedTime runtime  = new ElapsedTime();
    
    // Constructs the robot
    public RobotMap(){
        
    }
    
    // Initializes standard hardware interfaces
    public void init(HardwareMap hwMap) {
        
        // Defines and initializes all of the motors
        leftFrontDrive = hwMap.get(DcMotorEx.class, "leftFrontDrive");
        rightFrontDrive = hwMap.get(DcMotorEx.class, "rightFrontDrive");
        leftBackDrive = hwMap.get(DcMotorEx.class, "leftBackDrive");
        rightBackDrive = hwMap.get(DcMotorEx.class, "rightBackDrive");
        extender = hwMap.get(DcMotorEx.class, "extender");
        servo1 = hwMap.get(Servo.class, "servo1");
        servo2 = hwMap.get(Servo.class, "servo2");
        
        // Ensures the power is off on all the motors and the servos are closed
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        extender.setPower(0);
        servo1.setPosition(0.3);
        servo2.setPosition(0.2);
        
        // Tells each motor whether to run with or without an encoder (for the encoders: 1 rotation = 1440 ticks)
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        extender.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        extender.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
        // Sets the direction of all of the motors 
        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        extender.setDirection(DcMotorEx.Direction.REVERSE);
        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.FORWARD);
    }
}