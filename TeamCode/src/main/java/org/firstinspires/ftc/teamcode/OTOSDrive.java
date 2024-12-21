package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Collections;
import java.util.List;

@Autonomous(name = "OTOS Drive", group = "Auto")
public class OTOSDrive extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode() {

        // Auto Initialization Starts:

        robot.init(hardwareMap);

        Hardware.initializeDriveMotors(robot);

        waitForStart();
        resetRuntime();
        if (isStopRequested()) return;
        sleep(500);

        Pose2d startPose = robot.drive.getPoseEstimate();

        Trajectory myTrajectory = robot.drive.trajectoryBuilder(startPose)
                .forward(100)
                .build();

        robot.drive.followTrajectory(myTrajectory);

        telemetry.addData("Robot X", robot.odometrySensor.getPosition().x);
        telemetry.addData("Robot Y", robot.odometrySensor.getPosition().y);
        telemetry.addData("Robot H", robot.odometrySensor.getPosition().h);
        telemetry.update();

        while (opModeIsActive() && robot.drive.isBusy()) {
            robot.drive.update();
        }
    }
}
