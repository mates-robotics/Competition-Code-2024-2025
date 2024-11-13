package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;



@Autonomous(name = "AutoRedLeft")


public class AutoRedLeft extends LinearOpMode{


    /// Establishes the "robot" object from the "RobotMap" class
    RobotMap robot = new RobotMap();
    
    // Establishes the "runtime" object from the "ElapsedTime" class
    private ElapsedTime runtime = new ElapsedTime();
    
    @Override
    public void runOpMode() {
        
        // Initialize the hardware variables
        robot.init(hardwareMap);
        Methods methods = new Methods(robot);
        // Wait for driver to hit PLAY
        waitForStart();
        robot.servo1.setPosition(0.05);
        methods.strafe(-52, 1000);
        methods.drive(72, 1000);
        methods.extend_precise(-85);
        methods.strafe(28, 1000);
        //robot.extender.setVelocity(500000);
        methods.drive(10, 1000);
        
        
        //robot.extender.setTargetPosition(2117);
        //methods.extend_precise(-90);
        robot.servo1.setPosition(1);
        
        methods.drive(-10, 1000);
        robot.extender.setPower(0);
        methods.drive(10, 1000);

        robot.leftFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        robot.extender.setPower(0);
        robot.servo1.setPosition(0.3);


     }
}