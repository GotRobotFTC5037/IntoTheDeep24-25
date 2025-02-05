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
    private var sequenceRunning: Boolean = false
    private var actionStage = 0

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

        //sequence
        if (gamepad2.left_bumper && !sequenceRunning) {
            sequenceRunning = true
            actionStage = 0
            resetRuntime()
        }

        if (sequenceRunning) {
            if (robot.intakeGripper.position == robot.intakeGripperClosedTop) {
                when (actionStage) {
                    0 -> {
                        robot.deliveryPivot.position = robot.deliveryPivotLow
                        robot.deliveryGripper.position = robot.deliveryGripperOpen
                        robot.intakePivot.position = robot.intakePivotMid
                        resetRuntime()
                        actionStage++
                    }
                    1 -> {
                        if (runtime >= .3) {
                            robot.intakeWrist.position = robot.intakeWristLeft
                            robot.intakeSlide.position = robot.intakeSlideMid
                            resetRuntime()
                            actionStage++
                        }
                    }
                    2 -> {
                        if (runtime >= .3) {
                            robot.intakePivot.position = robot.intakePivotUp
                            resetRuntime()
                            actionStage++
                        }
                    }
                    3 -> {
                        if (runtime >= .4) {
                            robot.intakeSlide.position = robot.intakeSlideMin
                            resetRuntime()
                            actionStage++
                        }
                    }
                    4 -> {
                        if (runtime >= .3) {
                            robot.deliveryGripper.position = robot.deliveryGripperClosed
                            resetRuntime()
                            actionStage++
                        }
                    }
                    5 -> {
                        if (runtime >= .2) {
                            robot.intakeGripper.position = robot.intakeGripperNeutral
                            slidesAngle = robot.intakeSlideMin
                            wristAngle = robot.intakeWristLeft
                            actionStage++
                        }
                    }
                }
                if (actionStage == 6) {
                    sequenceRunning = false
                }
            } else {
                when (actionStage) {
                    0 -> {
                        robot.deliveryPivot.position = robot.deliveryPivotLow
                        robot.deliveryGripper.position = robot.deliveryGripperOpen
                        robot.intakePivot.position = robot.intakePivotMid
                        resetRuntime()
                        actionStage++
                    }
                    1 -> {
                        if (runtime >= 1) {
                            if (robot.intakeGripper.position ==  robot.intakeGripperClosedTop) {

                            }
                            robot.intakeGripper.position = robot.intakeGripperClosedSides
                            robot.intakePivot.position = robot.intakePivotUp
                        }
                    }
                }
            }
        }


        if (!sequenceRunning) {
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
                    wristAngle += (.005 * gamepad2.left_trigger)
                }
            } else if (gamepad2.right_trigger > 0.01) {
                if (wristAngle <= robot.intakeWristRight) {
                    wristAngle = robot.intakeWristRight
                } else {
                    wristAngle -= (.005 * gamepad2.right_trigger)
                }
            }


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

            aPressed = gamepad2.a

            if (robot.intakePivot.position == robot.intakePivotUp && robot.intakeSlide.position < .48) {
                wristAngle = if (robot.intakeGripper.position == robot.intakeGripperClosedTop) {
                    robot.intakeWristLeft
                } else {
                    robot.intakeWristMid
                }
            }


            robot.intakeWrist.position = wristAngle

            //Slides
            if (gamepad2.left_stick_y > 0.01) {
                if (slidesAngle >= robot.intakeSlideMin) {
                    slidesAngle = robot.intakeSlideMin
                } else {
                    slidesAngle += (.004 * gamepad2.left_stick_y)
                }
            } else if (gamepad2.left_stick_y < -0.01) {
                if (slidesAngle <= robot.intakeSlideMax) {
                    slidesAngle = robot.intakeSlideMax
                } else {
                    slidesAngle -= (.004 * abs(gamepad2.left_stick_y))
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
            if (robot.deliveryLiftDownSwitch.voltage < 2 ) {
                if (robot.intakeWrist.position != robot.intakeWristLeft) {
                    robot.intakeGripper.position = robot.intakeGripperClearance
                }
            }

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

        }


        // Distance Sensors
        telemetry.addData("gamepad2 dpad_right", gamepad2.dpad_right)
        telemetry.addData("Delivery Front Encoder", robot.deliveryFront.currentPosition)
        telemetry.addData("Delivery Back Encoder", robot.deliveryBack.currentPosition)
        telemetry.addData("Action Stage", actionStage)
        telemetry.addData("runtime", runtime)

//        telemetry.addData("Transfer color sensor", robot.transferDistanceSensor.red())
        telemetry.update()
    }

}
