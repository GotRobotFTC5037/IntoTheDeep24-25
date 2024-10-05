package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@TeleOp(name = "Tele", group="Robot")
public class Tele extends OpMode {
    Hardware robot = new Hardware();

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    SparkFunOTOS odometrySensor;

    @Override
    public void init() {

        telemetry.addData("Robot:", "Initializing");
        telemetry.update();

        robot.init(hardwareMap);

        robot.initializeDriveMotors(robot);

        telemetry.addData("Robot:", "Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        frontLeft.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x);
        frontRight.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x);
        backLeft.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x);
        backRight.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x);

        telemetry.addData("Stick X:", gamepad1.left_stick_x);
        telemetry.addData("Stick Y:", gamepad1.left_stick_y);

        telemetry.update();
    }
}
