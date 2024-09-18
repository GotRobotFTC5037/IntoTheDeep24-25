package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

@Autonomous(name="OurAuto")
class OurAuto : LinearOpMode() {

    lateinit var robot : FooRobot

    override fun runOpMode() {
        robot = FooRobot(hardwareMap)
        robot.run {
            setUp()
            waitForStart()
            driveForward(1000)
        }

    }



}