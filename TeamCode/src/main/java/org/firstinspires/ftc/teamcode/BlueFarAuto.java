package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Blue Far Auto", group = "Auto")
public class BlueFarAuto extends LinearOpMode {
    Hardware robot = new Hardware();
//    public SparkFunOTOS OTOS = robot.odometrySensor;
//    public SparkfunOdometryLocalizer localizer = new SparkfunOdometryLocalizer(OTOS);



    @Override
    public void runOpMode() {

        // Auto Initialization Starts:

        robot.init(hardwareMap);

        Hardware.initializeDriveMotors(robot);
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
//
//        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
//        drive.setPoseEstimate(startPose);

        waitForStart();
        resetRuntime();
        if (isStopRequested()) return;
        sleep(500);

        robot.escapement.setPosition(0);
        robot.kickstand.setPosition(0);
        robot.deliveryLiftMain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Auto Program Starts:

//        Pose2d currentPose = drive.getPoseEstimate();
//
//
//        TrajectorySequence toBars = drive.trajectorySequenceBuilder(startPose)
//                .forward(10)
//                .build();
//
//        drive.followTrajectorySequence(toBars);

        // Moves 24 inches exactly:
        robot.frontLeft.setPower(-0.25);
        robot.backLeft.setPower(-0.25);
        robot.frontRight.setPower(-0.25);
        robot.backRight.setPower(-0.25);
        sleep(1650);

        // Turns 90 degrees to the right
        robot.frontLeft.setPower(0.25);
        robot.backLeft.setPower(0.25);
        robot.frontRight.setPower(-0.25);
        robot.backRight.setPower(-0.25);
        sleep(2220);

        robot.frontLeft.setPower(-0.25);
        robot.backLeft.setPower(-0.25);
        robot.frontRight.setPower(-0.25);
        robot.backRight.setPower(-0.25);
        sleep(1650);

        robot.frontLeft.setPower(-0.25);
        robot.backLeft.setPower(-0.25);
        robot.frontRight.setPower(0.25);
        robot.backRight.setPower(0.25);
        sleep(1950);

        robot.frontLeft.setPower(-0.15);
        robot.backLeft.setPower(-0.15);
        robot.frontRight.setPower(-0.15);
        robot.backRight.setPower(-0.15);
        sleep(550);

        robot.deliveryLiftMain.setPower(-0.5);
        sleep(1200);

        robot.deliveryLiftMain.setPower(0.2);
        sleep(1950);

        robot.specimenGripper.setPosition(robot.specimenGripperUngrip);
        sleep(1000);

        robot.frontLeft.setPower(0.25);
        robot.backLeft.setPower(0.25);
        robot.frontRight.setPower(0.25);
        robot.backRight.setPower(0.25);
        sleep(2300);
    }
}
