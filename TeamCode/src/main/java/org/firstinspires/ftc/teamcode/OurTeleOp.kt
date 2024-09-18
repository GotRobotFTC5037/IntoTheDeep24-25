package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

@TeleOp(name = "OurTeleOp", group = "Concept")
class OurTeleOp : OpMode() {
    lateinit var robot : FooRobot

    override fun init() {
        robot = FooRobot(hardwareMap)
        robot.setUp()
    }

    override fun loop() {
        robot.frontLeft.power = (-gamepad1.left_stick_y + gamepad1.left_stick_x).toDouble()
        robot.frontRight.power = (-gamepad1.left_stick_y - gamepad1.left_stick_x).toDouble()
        robot.backLeft.power = (-gamepad1.left_stick_y - gamepad1.left_stick_x).toDouble()
        robot.backRight.power = (-gamepad1.left_stick_y + gamepad1.left_stick_x).toDouble()
        telemetry.addData("Stick X:", gamepad1.left_stick_x)
        telemetry.addData("Stick Y:", gamepad1.left_stick_y)
        telemetry.addData("liftPressed", robot.liftPressed)
        telemetry.update()

        robot.startIntake(gamepad2.right_bumper)

    }
}