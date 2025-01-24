package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "OTOSTestTele", group="Robot")
public class OTOSTestTele extends OpMode {
    Hardware robot = new Hardware();

    double y;
    double x;
    double rx;
    double denominator;
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;
    double speedLimit;
    public SparkFunOTOS OTOS = robot.odometrySensor;
    public SparkfunOdometryLocalizer localizer = new SparkfunOdometryLocalizer(OTOS);

    @Override
    public void init() {

        telemetry.addData("Robot:", "Initializing");
        telemetry.update();

        robot.init(hardwareMap);

        Hardware.initializeDriveMotors(robot);


    }

    public void start() {

        robot.frontLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.backLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.frontRight.setDirection(DcMotor.Direction.REVERSE);
        robot.backRight.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        // Forward/backward movement
        robot.frontLeft.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x);
        robot.frontRight.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x);
        robot.backLeft.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x);
        robot.backRight.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x);

        if (gamepad1.right_trigger > 0.5) {
            speedLimit = 50;
        } else {
            speedLimit = 100;
        }

        double speedLimitValue = speedLimit / 100;

        if (gamepad1.left_trigger > .5) {
            y = gamepad1.left_stick_y;
            x = -gamepad1.left_stick_x;
        } else {
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
        }
        rx = gamepad1.right_stick_x;

        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        frontLeftPower = (y + x + rx) / denominator;
        frontRightPower = (y - x - rx) / denominator;
        backLeftPower = (y - x + rx) / denominator;
        backRightPower = (y + x - rx) / denominator;

        if ((Math.abs(gamepad1.right_stick_x) > 0.1) || (Math.abs(gamepad1.right_stick_y) > 0.1) || (Math.abs(gamepad1.left_stick_x) > 0.1) || (Math.abs(gamepad1.left_stick_y) > 0.1)) {
            robot.frontLeft.setPower(frontLeftPower * speedLimitValue);
            robot.backLeft.setPower(backLeftPower * speedLimitValue);
            robot.frontRight.setPower(frontRightPower * speedLimitValue);
            robot.backRight.setPower(backRightPower * speedLimitValue);
        } else {
            robot.frontLeft.setPower(0);
            robot.backLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
        }

        telemetry.addData("OTOS (X value)", robot.odometrySensor.getPosition().x * robot.sensorMultipler);
        telemetry.addData("OTOS (Y value)", robot.odometrySensor.getPosition().y * robot.sensorMultipler);
        telemetry.addData("OTOS (H value)", robot.odometrySensor.getPosition().h);

        telemetry.update();
    }
}
