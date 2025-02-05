package org.firstinspires.ftc.teamcode

import com.qualcomm.ftcrobotcontroller.R
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor

@Autonomous(name = "Specimen Auto", group = "Auto")
class SpecimenAuto : LinearOpMode() {

    override fun runOpMode() {
        val robot = Robot(hardwareMap)

        resetRuntime()

        waitForStart()
        resetRuntime()
        if (isStopRequested) return
        sleep(500)



    }
}