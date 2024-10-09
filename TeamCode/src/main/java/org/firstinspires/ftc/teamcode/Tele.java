package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@TeleOp(name = "Tele", group="Robot")
public class Tele extends OpMode {
    Hardware robot = new Hardware();

    @Override
    public void init() {

        telemetry.addData("Robot:", "Initializing");
        telemetry.update();

        robot.init(hardwareMap);

        Hardware.initializeDriveMotors(robot);

        telemetry.addData("Robot:", "Ready");
        telemetry.update();
    }

    @Override
    public void start() {

        robot.odometrySensor.resetTracking();

    }

    @Override
    public void loop() {
        robot.frontLeft.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x);
        robot.frontRight.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x);
        robot.backLeft.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x);
        robot.backRight.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x);

        telemetry.addData("Stick X:", gamepad1.left_stick_x);
        telemetry.addData("Stick Y:", gamepad1.left_stick_y);

        telemetry.addData("OTOS (X value)", robot.odometrySensor.getPosition().x);
        telemetry.addData("OTOS (Y value)", robot.odometrySensor.getPosition().y);
        telemetry.addData("OTOS (H value)", robot.odometrySensor.getPosition().h);
        telemetry.addData("OTOS (Velocity X)", robot.odometrySensor.getVelocity().x);
        telemetry.addData("OTOS (Velocity Y)", robot.odometrySensor.getVelocity().y);
        telemetry.addData("OTOS (Velocity H)", robot.odometrySensor.getVelocity().h);
        telemetry.addData("OTOS (Acceleration X)", robot.odometrySensor.getAcceleration().x);
        telemetry.addData("OTOS (Acceleration Y)", robot.odometrySensor.getAcceleration().y);
        telemetry.addData("OTOS (Acceleration H)", robot.odometrySensor.getAcceleration().h);


        telemetry.update();
    }
}
