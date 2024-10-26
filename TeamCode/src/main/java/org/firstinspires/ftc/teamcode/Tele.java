package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@TeleOp(name = "Tele", group="Robot")
public class Tele extends OpMode {
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

        if (OTOS.isConnected() == false) {
            telemetry.addData("OTOS", "Not Connected");
            telemetry.update();
        }

        OTOS.begin();
        telemetry.addData("Calibrating IMU:", "Calibrating (1/2)");
        telemetry.update();

        OTOS.calibrateImu(100, true);
        telemetry.addData("Calibrating IMU:", "Calibrating (2/2)");
        telemetry.update();

        OTOS.setLinearScalar(1.0);
        OTOS.setAngularScalar(1.0);

        OTOS.resetTracking();

        telemetry.addData("OTOS", "Ready");
        telemetry.update();

        telemetry.addData("Robot", "Ready");
        telemetry.update();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        // Forward/backward movement
        robot.frontLeft.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x);
        robot.frontRight.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x);
        robot.backLeft.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x);
        robot.backRight.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x);

        // Turning movement
        if (gamepad1.right_trigger > .5) {
            speedLimit = 50;
        } else {
            speedLimit = 100;
        }

        double speedLimitValue = speedLimit / 100;


        if (gamepad1.left_trigger > .5) {
            y = gamepad1.left_stick_y;
            x = -gamepad1.left_stick_x * 1.1;
        } else {
            y = -gamepad1.left_stick_y; // Remember, this is reversed!
            x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
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

        //


        //


        telemetry.addData("Stick X:", gamepad1.left_stick_x);
        telemetry.addData("Stick Y:", gamepad1.left_stick_y);

        telemetry.addData("OTOS (X value)", robot.odometrySensor.getPosition().x);
        telemetry.addData("OTOS (Y value)", robot.odometrySensor.getPosition().y);
        telemetry.addData("OTOS (H value)", robot.odometrySensor.getPosition().h);
        telemetry.addData("OTOS (Velocity X)", robot.odometrySensor.getVelocity().x);
        telemetry.addData("OTOS (Velocity Y)", robot.odometrySensor.getVelocity().y);
        telemetry.addData("OTOS (Acceleration X)", robot.odometrySensor.getAcceleration().x);
        telemetry.addData("OTOS (Acceleration Y)", robot.odometrySensor.getAcceleration().y);


        telemetry.update();
    }
}
