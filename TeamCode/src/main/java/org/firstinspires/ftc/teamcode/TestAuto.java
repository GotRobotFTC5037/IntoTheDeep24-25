package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "TestAuto", group = "Auto")
public class TestAuto extends LinearOpMode {
    Hardware robot = new Hardware();
    public SparkFunOTOS OTOS = robot.odometrySensor;
    public SparkfunOdometryLocalizer localizer = new SparkfunOdometryLocalizer(OTOS);


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();


    }
}
