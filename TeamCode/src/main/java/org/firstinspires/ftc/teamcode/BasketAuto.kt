package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous(name = "Basket Auto", group = "Auto")
class BasketAuto : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(hardwareMap)

        robot.initializeInAuto()

        waitForStart()
        resetRuntime()
        if (isStopRequested) return
        sleep(500)
    }
}

