package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Test Specimen Auto", group = "Auto")
public class testSpecimenAuto extends LinearOpMode {

    private void TrajectoryActionBuilder() {

    }

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));

        waitForStart();
        resetRuntime();
        if (isStopRequested()) return;
        sleep(500);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(new Pose2d(0.0, 0.0, 0.0))
                .lineToX(12.0);

        Action trajectoryActionChosen;
        trajectoryActionChosen = tab1.build();

        telemetry.addData("Status", "Running trajectory...");
        telemetry.update();

                Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen
                )
        );


    }
}

