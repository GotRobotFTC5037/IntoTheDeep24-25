package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Motor Power Net Zone Auto", group = "Auto")
public class MotorPowerNetZone extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode() {

        // Auto Initialization Starts:

        robot.init(hardwareMap);

        Hardware.initializeDriveMotors(robot);

        robot.kickstand.setPosition(robot.kickstandDown);
        robot.deliveryLiftMain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        resetRuntime();
        if (isStopRequested()) return;
        sleep(500);

        // Code starts here
        // ALL CODE ASSUMES SPECIMEN SIDE OF ROBOT IS FRONT (according to starting position in auto)

        diagonalStrafe(0.15, -0.5, 2);
        sleep(100);
        robot.deliveryLiftMain.setPower(-0.775);
        sleep(750);
        driveForward(0.2, 1);
        robot.deliveryLiftMain.setPower(0.2);
        sleep(1400);
        diagonalStrafe(-0.45, -0.875, 2);
        sleep(100);
        turn180Degrees(0.35, 3);
        sleep(100);
        robot.specimenGripper.setPosition(robot.specimenGripperUngrip);
        driveForward(0.2, 2);
        sleep(200);
        robot.specimenGripper.setPosition(0.36);
        sleep(750);
        robot.deliveryLiftMain.setPower(-0.35);
        sleep(750);
        driveForward(-0.2, 1);
        turn180Degrees(-0.345, 3);
        diagonalStrafe(0.235, 0.775, 2);
        robot.deliveryLiftMain.setPower(-0.9);
        sleep(900);
        robot.deliveryLiftMain.setPower(0.2);
        sleep(1400);
        robot.specimenGripper.setPosition(robot.specimenGripperUngrip);
        diagonalStrafe(-0.55, -0.95, 2);
        sleep(500);
    }
    public void driveForward(double motorPower, int timeInSeconds) {
        // Assuming you have access to motor objects (motor1, motor2, motor3, motor4)
        // Set all motors to the same power for forward motion
        robot.frontLeft.setPower(-motorPower);
        robot.frontRight.setPower(-motorPower);
        robot.backLeft.setPower(-motorPower);
        robot.backRight.setPower(-motorPower);

        // Wait for the given period (convert seconds to milliseconds)
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < timeInSeconds * 1000) {
            // Keep the motors running for the specified time
            // Optionally, you can add a sleep to reduce the CPU usage
            // Thread.sleep(10); // Uncomment if needed, but use it with care
        }

        // Stop all motors after the time has passed
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
    }

    public void diagonalStrafe(double forwardPower, double strafePower, int timeInSeconds) {
        // Assuming positive forwardPower moves the robot forward
        // Assuming positive strafePower moves the robot to the right
        // The combination of both movements results in a diagonal path.

        // Motor assignments for diagonal movement (forward and right)
        robot.frontLeft.setPower(-forwardPower + strafePower);  // Front-left motor (forward + right)
        robot.frontRight.setPower(-forwardPower - strafePower);  // Front-right motor (forward - right)
        robot.backLeft.setPower(-forwardPower - strafePower);  // Back-left motor (forward - right)
        robot.backRight.setPower(-forwardPower + strafePower);  // Back-right motor (forward + right)

        // Wait for the given period (convert seconds to milliseconds)
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < timeInSeconds * 1000) {
            // Keep the motors running for the specified time
        }

        // Stop all motors after the time has passed
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
    }

    public void turn180Degrees(double turnPower, int timeInSeconds) {
        // Motor assignments for a 180-degree turn (rotate around the robot's center).
        // The left motors move forward, and the right motors move backward.
        robot.frontLeft.setPower(turnPower);   // Front-left motor moves forward
        robot.backLeft.setPower(turnPower);   // Back-left motor moves forward
        robot.frontRight.setPower(-turnPower);  // Front-right motor moves backward
        robot.backRight.setPower(-turnPower);  // Back-right motor moves backward

        // Wait for the robot to turn for the given period (convert seconds to milliseconds)
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < timeInSeconds * 1000) {
            // Continue turning for the specified time
        }

        // Stop all motors after the time has passed
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
    }


}
