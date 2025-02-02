package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@TeleOp(name = "Tele", group="Robot")
class Tele : OpMode() {

    private lateinit var robot: Robot

    private var deliveryGripperTime: Double = 0.0
    private var deliveryGripperPosition: Double = 0.0
    private var gripperState: Int = 0
    private var wristAngle = 0.39


    override fun init() {
        robot = Robot(hardwareMap = hardwareMap)

        resetRuntime()
    }

    override fun loop() {
        robot.moveRobot(
            x = gamepad1.left_stick_x.toDouble(),
            y = -gamepad1.left_stick_y.toDouble(),
            yaw = -gamepad1.right_stick_x.toDouble()
        )

        // Intake Gripper Movement
        if (gamepad2.b) {
            robot.intakeGripper.position = robot.intakeGripperClosedLoose
        }
        if (gamepad2.x) {
            robot.intakeGripper.position = robot.intakeGripperNeutral
        }
        if (gamepad2.y) {
            robot.intakeGripper.position = robot.intakeGripperClosedTop
        }

        if (gamepad2.right_trigger > 0.1) {
            if (wristAngle >= 0.8) {
                wristAngle = 0.8
            } else {
                wristAngle = wristAngle + (.005 * gamepad2.right_trigger)
            }
        } else if (gamepad2.left_trigger > 0.1) {
            if (wristAngle <= 0.15) {
                wristAngle = 0.2
            } else {
                wristAngle = wristAngle - (.005 * gamepad2.left_trigger)
            }
        }

        robot.intakeWrist.setPosition(wristAngle);


        // Specimen Gripper Movement
        if (gamepad2.right_bumper) {
            robot.specimenGripper.position = robot.specimenGripperOpen
        } else {
            robot.specimenGripper.position = robot.specimenGripperClosed
        }

        // Delivery Gripper
        if (gamepad2.dpad_down) {
            robot.deliveryPivot.position = robot.deliveryPivotLow
        }
        if (gamepad2.dpad_left) {
            robot.deliveryPivot.position = robot.deliveryPivotMedium
        }
        if (gamepad2.dpad_up) {
            robot.deliveryPivot.position = robot.deliveryPivotHigh
        }

        when(gripperState) {
            0 -> if (gamepad2.dpad_right) {
                gripperState = 1
            }

            1 -> if (!gamepad2.dpad_right) {
                gripperState = 2
            }

            2 -> if (robot.deliveryGripper.position == robot.deliveryGripperOpen) {
                robot.deliveryGripper.position = robot.deliveryGripperClosed
            } else {
                robot.deliveryGripper.position = robot.deliveryGripperOpen
            }

        }

        if (gripperState == 2) {
            gripperState = 0
        }

        // Distance Sensors
        telemetry.addData("gripper state", gripperState)
        telemetry.addData("gamepad2 dpad_right", gamepad2.dpad_right)
//        telemetry.addData("Transfer color sensor", robot.transferDistanceSensor.red())
        telemetry.update()
    }

}
