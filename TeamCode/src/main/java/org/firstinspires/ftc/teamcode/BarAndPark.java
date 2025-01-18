//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
//@Autonomous(name = "Bar and Park", group = "Auto")
//public class BarAndPark  extends LinearOpMode {
//    Hardware robot = new Hardware();
//
//    @Override
//    public void runOpMode() {
//
//        // Auto Initialization Starts:
//
//        robot.init(hardwareMap);
//
//        Hardware.initializeDriveMotors(robot);
//
//        waitForStart();
//        resetRuntime();
//        if (isStopRequested()) return;
//        sleep(500);
//
//        robot.escapement.setPosition(0);
//        robot.kickstand.setPosition(0);
//        robot.deliveryLiftMain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
////        robot.frontLeft.setTargetPosition(900);
////        robot.frontRight.setTargetPosition(900);
////        robot.backLeft.setTargetPosition(900);
////        robot.backRight.setTargetPosition(900);
////
////        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
//        robot.drive.setMotorPowers(-0.3, -0.3, -0.3, -0.3);
//        sleep(1000);
//
//        robot.specimenGripper.setPosition(0.38);
//
//        robot.deliveryLiftMain.setPower(-.75);
//        robot.drive.setMotorPowers(0, 0, 0, 0);
//        sleep(1500);
//        sleep(250);
//
//        robot.deliveryLiftMain.setPower(-0.1);
//        robot.drive.setMotorPowers(-0.2,-0.2,-0.2,-0.2);
//        sleep(1700);
//
//        robot.drive.setMotorPowers(0,0,0,0);
//        sleep(2000);
//
//        robot.deliveryLiftMain.setPower(.0001);
//        sleep(250);
//
//        robot.specimenGripper.setPosition(0);
//        sleep(1000);
//
//        robot.drive.setMotorPowers(0.2,0.2,0.2,0.2);
//        sleep(1500);
//
//        robot.frontLeft.setPower(-0.25);
//        robot.backLeft.setPower(-0.25);
//        robot.frontRight.setPower(0.25);
//        robot.backRight.setPower(0.25);
//        sleep(1110);
//
//        robot.drive.setMotorPowers(0.2,0.2,0.2,0.2);
//        sleep(2500);
//    }
//}
