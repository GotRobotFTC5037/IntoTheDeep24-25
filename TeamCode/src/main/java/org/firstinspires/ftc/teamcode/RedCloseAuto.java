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

@Autonomous(name = "Red Close Auto", group = "Auto")
public class RedCloseAuto extends LinearOpMode {
    Hardware robot = new Hardware();
    public SparkFunOTOS OTOS = robot.odometrySensor;
    public SparkfunOdometryLocalizer localizer = new SparkfunOdometryLocalizer(OTOS);



    @Override
    public void runOpMode() {

        // Auto Initialization Starts:

        robot.init(hardwareMap);

        Hardware.initializeDriveMotors(robot);

        robot.backRight.setDirection(DcMotorSimple.Direction.FORWARD);

//        drive = new SparkfunOdometryLocalizer(OTOS);
//        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


//        if (!OTOS.isConnected()) {
//            telemetry.addData("OTOS", "Not Connected");
//            telemetry.update();
//        }
//
//        OTOS.begin();
//        telemetry.addData("Calibrating IMU:", "Calibrating (1/2)");
//        telemetry.update();
//
//        OTOS.calibrateImu(100, true);
//        telemetry.addData("Calibrating IMU:", "Calibrating (2/2)");
//        telemetry.update();
//
//        OTOS.setLinearScalar(1.0);
//        OTOS.setAngularScalar(1.0);
//
//        OTOS.resetTracking();
//
//        telemetry.addData("OTOS", "Ready");
//        telemetry.update();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
//        drive.setPoseEstimate(startPose);

        waitForStart();
        resetRuntime();
        if (isStopRequested()) return;
        sleep(500);

//        robot.escapement.setPosition(0);
//        robot.kickstand.setPosition(0);
//        robot.deliveryLiftMain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        otosDrive(2, 7, 8, 2);
//
//        double targetX = 0;
//        double targetY = 0;
//        double targetHeading = 0;
//        int maxTime = 0;
//        otosDrive(targetX, targetY, targetHeading, maxTime);

        // Auto Program Starts:

//        // Moves 24 inches exactly:
        robot.frontLeft.setPower(-0.5);
        robot.backLeft.setPower(-0.5);
        robot.frontRight.setPower(-0.5);
        robot.backRight.setPower(-0.5);
        sleep(2000);
//
//        // Turns 90 degrees to the right
//        robot.frontLeft.setPower(0.25);
//        robot.backLeft.setPower(0.25);
//        robot.frontRight.setPower(-0.25);
//        robot.backRight.setPower(-0.25);
//        sleep(2220);
//
//        robot.frontLeft.setPower(-0.25);
//        robot.backLeft.setPower(-0.25);
//        robot.frontRight.setPower(-0.25);
//        robot.backRight.setPower(-0.25);
//        sleep(1650);
//
//        robot.frontLeft.setPower(-0.25);
//        robot.backLeft.setPower(-0.25);
//        robot.frontRight.setPower(0.25);
//        robot.backRight.setPower(0.25);
//        sleep(1950);
//        robot.frontLeft.setPower(0);
//        robot.backLeft.setPower(0);
//        robot.frontRight.setPower(0);
//        robot.backRight.setPower(0);
//        sleep(1000);
//        robot.deliveryLiftMain.setPower(-0.5);
//        sleep(1100);
//        robot.frontLeft.setPower(-0.15);
//        robot.backLeft.setPower(-0.15);
//        robot.frontRight.setPower(-0.15);
//        robot.backRight.setPower(-0.15);
//        sleep(400);
//        robot.deliveryLiftMain.setPower(0.2);
//        sleep(1950);
////        robot.specimenGripper.setPosition(robot.specimenGripperUngrip);
////        sleep(1000);
//
//        robot.frontLeft.setPower(0.25);
//        robot.backLeft.setPower(0.25);
//        robot.frontRight.setPower(0.25);
//        robot.backRight.setPower(0.25);
//        sleep(2500);
    }

//    private void otosDrive(double targetX, double targetY, double targetHeading, int maxTime) {
//    }
}
