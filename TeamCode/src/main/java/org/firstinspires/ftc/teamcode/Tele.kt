package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap

@TeleOp(name = "Tele", group="Robot")
class Tele : OpMode() {

    private lateinit var robot: Robot

    override fun init() {
        robot = Robot(hardwareMap = hardwareMap)
    }

    override fun loop() {
        robot.moveRobot(
            x = gamepad1.left_stick_x.toDouble(),
            y = -gamepad1.left_stick_y.toDouble(),
            yaw = -gamepad1.right_stick_x.toDouble()
        )



    }

}
