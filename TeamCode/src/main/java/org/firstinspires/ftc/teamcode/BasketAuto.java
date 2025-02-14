package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Basket Auto", group = "Auto")
public class BasketAuto extends LinearOpMode {

    private void TrajectoryActionBuilder() {

    }

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
        robot.getIntakeSlide().setPosition(robot.getIntakeSlideMin());
        robot.getDeliveryPivot().setPosition(robot.getDeliveryPivotLow());
        robot.getIntakeWrist().setPosition(robot.getIntakeWristLeft());
        robot.getDeliveryGripper().setPosition(robot.getDeliveryGripperClosed());
        robot.getIntakeGripper().setPosition(robot.getIntakeGripperNeutral());
        waitForStart();
        resetRuntime();
        if (isStopRequested()) return;
        sleep(500);

//        robot.getIntakeSlide().setPosition(robot.getIntakeSlideMin());
//
        /*TrajectoryActionBuilder deliver = drive.actionBuilder(new Pose2d(0.0, 0.0, 0.0))
//                .strafeTo(new Vector2d(4.5,4.5))
//                .waitSeconds(2);
                .lineToX(8.0)
                .setTangent(Math.toRadians(90))
                .lineToY(12.0)
                .waitSeconds(1);
//                .setTangent(Math.toRadians(10))
//                .lineToX(10.0)
//                .waitSeconds(1);


        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(0.0,0.0,0.0))
                .turn(45)
                .lineToX(-10.0)
                .setTangent(Math.toRadians(90))
                .lineToY(-15.0);


        Action trajectoryActionChosen;
        trajectoryActionChosen = deliver.build();*/

//        Action trajectoryActionChosen2;
//        trajectoryActionChosen2 = park.build();
//
//                Actions.runBlocking(
//                new SequentialAction(
//                        trajectoryActionChosen
////                        trajectoryActionChosen2
//
//                )
//        );

//        telemetry.addData("position x", drive.localizer.getPose().position.x);
//        telemetry.addData("position x", drive.localizer.getPose().position.y);

       robot.moveRobot(0.0,0.155,0.0);
       sleep(725);

       robot.getDeliveryPivot().setPosition(robot.getDeliveryPivotMedium());

       robot.moveLiftToPosition(2400,0.5);

       sleep(1200);

       robot.getDeliveryGripper().setPosition(robot.getDeliveryGripperOpen());
       sleep(1000);

        robot.getDeliveryBack().setPower(0.5);
        robot.getDeliveryFront().setPower(0.5);

        robot.getDeliveryPivot().setPosition(robot.getDeliveryPivotHigh());

       sleep(500);


        sleep(1000);

        robot.getIntakeWrist().setPosition(robot.getIntakeWristLeft());
        robot.getIntakeGripper().setPosition(robot.getIntakeGripperClosedLoose());
        robot.getIntakePivot().setPosition(robot.getIntakePivotUp());

        robot.getDeliveryPivot().setPosition(robot.getDeliveryPivotLow());
        robot.getDeliveryGripper().setPosition(robot.getDeliveryGripperOpen());
        robot.getIntakeSlide().setPosition(robot.getIntakeSlideMin());


//       robot.getDeliveryPivot().setPosition(robot.getDeliveryPivotMedium());
//       robot.getDeliveryGripper().setPosition(robot.getDeliveryGripperOpen());
//       sleep(300);
//       robot.moveDiagonally(0.85,500,"backward_right");
//       sleep(300);
//       robot.turnRobot(0.5,200);
//       sleep(300);
//       robot.moveLiftToPosition(2400,0.5);
//       sleep(5000);


    }
}

