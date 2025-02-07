package org.firstinspires.ftc.teamcode


import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.MecanumDrive.FollowTrajectoryAction
import com.acmerobotics.roadrunner.Trajectory
import com.acmerobotics.roadrunner.TrajectoryActionFactory
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.TimeTrajectory




@Autonomous(name = "Specimen Auto", group = "Auto")
class SpecimenAuto : LinearOpMode() {

//    private fun TrajectoryActionBuilder() {
//
//    }

    override fun runOpMode() {
        val robot = Robot(hardwareMap)
        val drive = MecanumDrive(hardwareMap,Pose2d(0.0,0.0,0.0))

        waitForStart()
        resetRuntime()
        if (isStopRequested) return
        sleep(500)

       drive.actionBuilder(Pose2d(0.0,0.0,0.0))
                .lineToX(-12.0)



//        robot.moveRobot(-0.5,0.5,0.0)
//        sleep(800)
//        robot.moveRobot(0.0,0.15,0.0)
//        sleep(500)
//        robot.moveRobot(0.0,0.0,0.8)
//        sleep(1100)
//        robot.moveRobot(-0.5,0.25,0.0)
//        sleep(1000)
//        robot.moveForward(0.5,0.5,0.0,0.0)
//        sleep(1000)

    }



}
