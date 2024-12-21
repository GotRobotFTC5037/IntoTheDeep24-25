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

@Autonomous(name = "Left to Park Auto", group = "Auto")
public class LeftToPark  extends LinearOpMode {
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

        robot.frontLeft.setPower(-0.3);
        robot.backLeft.setPower(-0.3);
        robot.frontRight.setPower(-0.3);
        robot.backRight.setPower(-0.3);
        sleep(3500);
    }
}
