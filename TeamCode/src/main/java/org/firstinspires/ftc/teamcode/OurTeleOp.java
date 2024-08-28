package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Concept: NullOp", group = "Concept")
public class OurTeleOp extends OpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    @Override
    public void init() {

        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

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
