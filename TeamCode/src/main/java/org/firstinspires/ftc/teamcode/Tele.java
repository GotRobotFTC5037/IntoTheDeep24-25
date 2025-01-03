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
//
//        telemetry.addData("Robot", "Ready");
//        telemetry.update();
    }

    public void start() {
//        robot.escapement.setPosition(0);
//        robot.kickstand.setPosition(0);
//        robot.hookRelease.setPosition(0);
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

        int maxArmPosition = 2350;
        double intakeArmSpeedLimit = 0.8;
        robot.intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (((gamepad2.right_stick_y > 0) || (gamepad2.right_stick_y < 0 && robot.intakeArm.getCurrentPosition() <= maxArmPosition))) {
            robot.intakeArm.setPower(gamepad2.right_stick_y * intakeArmSpeedLimit);
        } else if (robot.intakeLimitSwitch.getVoltage() < 2) {
            robot.intakeArm.setPower(0);
            robot.intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            robot.intakeArm.setPower(0);
        }

//        int maxArmPosition = 2150;
//        robot.intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        // Intake slide
//        if (((gamepad2.right_stick_y > 0 && robot.intakeLimitSwitch.getVoltage() < 2) || (gamepad2.right_stick_y < 0 && robot.intakeArm.getCurrentPosition() < maxArmPosition)) && (robot.intakeArm.getCurrentPosition() < maxArmPosition)) {
//            robot.intakeArm.setPower(gamepad2.right_stick_y);
//        } else {
//            robot.intakeArm.setPower(0);

//        }
//        if (robot.intakeLimitSwitch.getVoltage() > 2) {
//            robot.intakeArm.setPower(0);
//            robot.intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }

        // Intake stars
//        robot.stars.setPower(gamepad2.right_trigger);
//        if (gamepad2.right_bumper) {
//            robot.stars.setPower(-1);
//        }

        if (gamepad2.right_trigger > 0.5) {
            robot.stars.setPower(1);
        } else if (gamepad2.right_bumper) {
            robot.stars.setPower(-1);
        } else {
            robot.stars.setPower(0.0);
        }

        double downwardsSpeedLimit = 0.25;
        // Delivery lift

         if (gamepad2.left_stick_y < 0 && robot.mainLiftUpLimitSwitch.getVoltage() > 2) {
             robot.deliveryLiftMain.setPower(gamepad2.left_stick_y);

         } else if (gamepad2.left_stick_y < 0 && robot.auxLiftUpLimitSwitch.getVoltage() > 2) {
             robot.deliveryLiftAux.setPower(-gamepad2.left_stick_y);

         } else if (gamepad2.left_stick_y > 0 && robot.auxLiftDownLimitSwitch.getVoltage() > 2) {
             if (gamepad2.a) {
                 robot.deliveryLiftAux.setPower(-gamepad2.left_stick_y * 0.05);
             } else {
                 robot.deliveryLiftAux.setPower(-gamepad2.left_stick_y * downwardsSpeedLimit);
             }
         } else if (gamepad2.left_stick_y > 0 && robot.mainLiftDownLimitSwitch.getVoltage() > 2) {
             if (gamepad2.a) {
                 robot.deliveryLiftMain.setPower(gamepad2.left_stick_y * 0.05);
             } else {
                 robot.deliveryLiftMain.setPower(gamepad2.left_stick_y * downwardsSpeedLimit);
             }
         } else {
             robot.deliveryLiftMain.setPower(0);
             robot.deliveryLiftAux.setPower(0);
         }

        if (gamepad2.dpad_up || (robot.mainLiftDownLimitSwitch.getVoltage() > 2)) {
            robot.bucket.setPosition(0.6);
        } else {
            robot.bucket.setPosition(0);
        }

        if (gamepad2.y) {
            robot.escapement.setPosition(0.5);
        } else {
            robot.escapement.setPosition(0);
        }

        if (gamepad2.left_bumper) {
            robot.specimenGripper.setPosition(0);
        }
        if (gamepad2.left_trigger > 0.5) {
            robot.specimenGripper.setPosition(0.36);
        }
        if (gamepad2.dpad_left) {
            robot.kickstand.setPosition(0.5);
        }

//        Escapement
        if (gamepad2.y) {
            robot.escapement.setPosition(0.8);
        }

//         Specimen Gripper
        if (gamepad2.left_bumper) {
            robot.specimenGripper.setPosition(robot.specimenGripperUngrip);
        }

        if (gamepad1.dpad_up) {
            robot.lift.setPower(-0.5);
        } else if (gamepad1.dpad_down) {
            robot.lift.setPower(0.5);
        } else {
            robot.lift.setPower(0);
        }

        if (gamepad1.dpad_right) {
            robot.hookRelease.setPosition(0.35);
        } else {
            robot.hookRelease.setPosition(.47);
        }

        telemetry.addData("Hook release position", robot.hookRelease.getPosition());
//        telemetry.addData("Right trigger value", gamepad2.right_trigger);
//        telemetry.addData("Right bumper value", gamepad2.right_bumper);
//        telemetry.addData("Stars power", robot.stars.getPower());
//        telemetry.addData("Right stick y", gamepad2.right_stick_y);
//        telemetry.addData("Intake arm position", robot.intakeArm.getCurrentPosition());
//        telemetry.addData("Intake switch voltage", robot.intakeLimitSwitch.getVoltage());
//        telemetry.addData("Bucket Position", robot.bucket.getPosition());
//        telemetry.addData("Main lift down switch voltage", robot.mainLiftDownLimitSwitch.getVoltage());
//        telemetry.addData("Stick X:", gamepad1.left_stick_x);
//        telemetry.addData("Stick Y:", gamepad1.left_stick_y);
//
//        telemetry.addData("Intake Switch", robot.intakeLimitSwitch.getVoltage());
//        telemetry.addData("Main lift up Switch", robot.mainLiftUpLimitSwitch.getVoltage());
//        telemetry.addData("Main lift down Switch", robot.mainLiftDownLimitSwitch.getVoltage());
//        telemetry.addData("Aux lift up Switch", robot.auxLiftUpLimitSwitch.getVoltage());
//        telemetry.addData("Aux lift down Switch", robot.auxLiftDownLimitSwitch.getVoltage());

//        telemetry.addData("OTOS (X value)", robot.odometrySensor.getPosition().x);
//        telemetry.addData("OTOS (Y value)", robot.odometrySensor.getPosition().y * robot.LINEAR_SCALAR);
//        telemetry.addData("OTOS (H value)", robot.odometrySensor.getPosition().h);
//        telemetry.addData("OTOS (Velocity X)", robot.odometrySensor.getVelocity().x);
//        telemetry.addData("OTOS (Velocity Y)", robot.odometrySensor.getVelocity().y);
//        telemetry.addData("OTOS (Acceleration X)", robot.odometrySensor.getAcceleration().x);
//        telemetry.addData("OTOS (Acceleration Y)", robot.odometrySensor.getAcceleration().y);

//        telemetry.addData("Bucket Position", robot.bucket.getPosition());
//        telemetry.addData("Gripper Position", robot.specimenGripper.getPosition());
//        telemetry.addData("Intake Arm Switch", robot.intakeLimitSwitch.getVoltage());
//        telemetry.addData("Intake Arm Power", robot.intakeArm.getPower());
//        telemetry.addData("Intake Arm Encoder Output", robot.intakeArm.getCurrentPosition());

        telemetry.update();
    }
}
