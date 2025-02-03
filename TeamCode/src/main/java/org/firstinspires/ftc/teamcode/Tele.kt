package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.abs

@TeleOp(name = "Tele", group="Robot")
class Tele : OpMode() {

    private lateinit var robot: Robot

    private var dpadRightPressed: Boolean = false
    private var aPressed: Boolean = false
    private var wristAngle = 0.475
    private var slidesAngle = 0.54


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

        //Intake Movement
        //Gripper
        if (gamepad2.b) {
            robot.intakeGripper.position = robot.intakeGripperClosedLoose
        }
        if (gamepad2.x) {
            robot.intakeGripper.position = robot.intakeGripperNeutral
        }
        if (gamepad2.y) {
            robot.intakeGripper.position = robot.intakeGripperClosedTop
        }

        //Wrist
        if (gamepad2.left_trigger > 0.01) {
            if (wristAngle >= robot.intakeWristLeft) {
                wristAngle = robot.intakeWristLeft
            } else {
                wristAngle += (.003 * gamepad2.left_trigger)
            }
        } else if (gamepad2.right_trigger > 0.01) {
            if (wristAngle <= robot.intakeWristRight) {
                wristAngle = robot.intakeWristRight
            } else {
                wristAngle -= (.003 * gamepad2.right_trigger)
            }
        }

        robot.intakeWrist.setPosition(wristAngle)

        //Pivot
        if (!aPressed) {
            if (gamepad2.a) {
                if (robot.intakePivot.position == robot.intakePivotUp) {
                    robot.intakePivot.position = robot.intakePivotDown
                } else {
                    robot.intakePivot.position = robot.intakePivotUp
                }
            }
        }

        if (gamepad2.a) {
            aPressed = true
        } else {
            aPressed = false
        }

//        if (robot.intakePivot.position == robot.intakePivotUp) {
//            if (robot.intakeGripper.position == robot.intakeGripperClosedTop) {
//                robot.intakeWrist.position = robot.intakeWristLeft
//            } else {
//                robot.intakeWrist.position = robot.intakeWristMid
//            }
//        }

        //Slides
        if (gamepad2.left_stick_y > 0.01) {
            if (slidesAngle >= robot.intakeSlideMin) {
                slidesAngle = robot.intakeSlideMin
            } else {
                slidesAngle += (.002 * gamepad2.left_stick_y)
            }
        } else if (gamepad2.left_stick_y < -0.01) {
            if (slidesAngle <= robot.intakeSlideMax) {
                slidesAngle = robot.intakeSlideMax
            } else {
                slidesAngle -= (.002 * abs(gamepad2.left_stick_y))
            }
        }
        robot.intakeSlide.position = slidesAngle

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

        if (!dpadRightPressed) {
            if (gamepad2.dpad_right) {
                if (robot.deliveryGripper.position == robot.deliveryGripperOpen) {
                    robot.deliveryGripper.position = robot.deliveryGripperClosed
                } else {
                    robot.deliveryGripper.position = robot.deliveryGripperOpen
                }
            }
        }

        if (gamepad2.dpad_right) {
            dpadRightPressed = true
        } else {
            dpadRightPressed = false
        }

        //Delivery Slides needs encoder
        if (gamepad2.right_stick_y > 0.1 && robot.deliveryLiftDownSwitch.voltage < 2 ) {
            robot.deliveryFront.power = gamepad2.right_stick_y.toDouble()
            robot.deliveryBack.power = gamepad2.right_stick_y.toDouble()
        } else if (gamepad2.right_stick_y < 0.1 ) {
            robot.deliveryFront.power = gamepad2.right_stick_y.toDouble()
            robot.deliveryBack.power = gamepad2.right_stick_y.toDouble()
        } else {
            robot.deliveryFront.power = 0.0
            robot.deliveryBack.power = 0.0
        }


        // Distance Sensors
        telemetry.addData("gamepad2 dpad_right", gamepad2.dpad_right)
        telemetry.addData("votage", robot.deliveryLiftDownSwitch.voltage)
//        telemetry.addData("Transfer color sensor", robot.transferDistanceSensor.red())
        telemetry.update()
    }

}
