package org.firstinspires.ftc.teamcode

import android.os.Parcel
import android.os.Parcelable
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import kotlin.math.abs

@TeleOp(name = "Tele", group="Robot")
class Tele() : OpMode() {

    private lateinit var robot: Robot

    private var dpadRightPressed: Boolean = false
    private var aPressed: Boolean = false
    private var wristAngle = 0.475
    private var slidesAngle = 0.37
    private var sequenceRunning: Boolean = false
    private var actionStage = 0
    private var deliveryHeightGoal = 2500
    private var resetEncoder: Boolean = false
    private var allowWristMove: Boolean = true

    constructor(parcel: Parcel) : this() {
        dpadRightPressed = parcel.readByte() != 0.toByte()
        aPressed = parcel.readByte() != 0.toByte()
        wristAngle = parcel.readDouble()
        slidesAngle = parcel.readDouble()
        sequenceRunning = parcel.readByte() != 0.toByte()
        actionStage = parcel.readInt()
    }

    override fun init() {
        robot = Robot(hardwareMap = hardwareMap)


        resetRuntime()
    }

    override fun start() {
        robot.intakeWrist.position = robot.intakeWristLeft
        robot.intakeGripper.position = robot.intakeGripperClosedLoose
        robot.intakePivot.position = robot.intakePivotUp

        robot.deliveryPivot.position = robot.deliveryPivotLow
        robot.deliveryGripper.position = robot.deliveryGripperOpen


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
                        if (runtime >= .45) {
                            robot.intakeSlide.position = robot.intakeSlideMin + 0.05
                            resetRuntime()
                            actionStage++
                        }
                    }

                    4 -> {
                        if (runtime >= .45) {
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
            } else {
                when (actionStage) {
                    0 -> {
                        robot.intakeWrist.position = robot.intakeWristMid
                        robot.deliveryPivot.position = robot.deliveryPivotLow
                        robot.deliveryGripper.position = robot.deliveryGripperOpen
                        robot.intakePivot.position = robot.intakePivotMid
                        resetRuntime()
                        actionStage++
                    }

                    1 -> {
                        if (runtime >= .7) {
                            robot.intakeSlide.position = robot.intakeSlideMid
                            resetRuntime()
                            actionStage++
                        }
                    }

                    2 -> {
                        if (runtime >= .3) {
                            robot.intakeGripper.position = robot.intakeGripperClosedSides
                            robot.intakePivot.position = robot.intakePivotUp
                            resetRuntime()
                            actionStage++
                        }
                    }

                    3 -> {
                        if (runtime >= .45) {
                            robot.intakeSlide.position = robot.intakeSlideMin
                            resetRuntime()
                            actionStage++
                        }
                    }

                    4 -> {
                        if (runtime >= .45) {
                            robot.deliveryGripper.position = robot.deliveryGripperClosed
                            resetRuntime()
                            actionStage++
                        }
                    }

                    5 -> {
                        if (runtime >= .2) {
                            robot.intakeGripper.position = robot.intakeGripperClearance
                            slidesAngle = robot.intakeSlideMin
                            wristAngle = robot.intakeWristMid
                            actionStage++
                        }
                    }
                }
            }
        }

        if (actionStage == 6) {
            sequenceRunning = false
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
                if (wristAngle <= robot.intakeWristLeft) {
                    wristAngle = robot.intakeWristLeft
                } else {
                    wristAngle -= (.01 * gamepad2.left_trigger)
                }
            } else if (gamepad2.right_trigger > 0.01) {
                if (wristAngle >= robot.intakeWristRight) {
                    wristAngle = robot.intakeWristRight
                } else {
                    wristAngle += (.01 * gamepad2.right_trigger)
                }
            }


            //Pivot
            if (!aPressed) {
                if (gamepad2.a) {
                    if (robot.intakePivot.position == robot.intakePivotUp) {
                        robot.intakePivot.position = robot.intakePivotDown
                        robot.intakeGripper.position = robot.intakeGripperNeutral
                        wristAngle = robot.intakeWristMid
                        robot.intakeWrist.position = robot.intakeWristMid
                    } else {
                        robot.intakePivot.position = robot.intakePivotUp
                        robot.intakeWrist.position = robot.intakeWristLeft
                    }
                }
            }
            aPressed = gamepad2.a


            if (gamepad2.left_trigger > 0.01 || gamepad2.right_trigger > 0.01) {
                robot.intakeWrist.position = wristAngle
            }

            //Slides
            if (gamepad2.left_stick_y > 0.01) {
                if (slidesAngle <= robot.intakeSlideMin) {
                    slidesAngle = robot.intakeSlideMin
                } else {
                    slidesAngle -= (.005 * gamepad2.left_stick_y)
                }
            } else if (gamepad2.left_stick_y < -0.01) {
                if (slidesAngle >= robot.intakeSlideMax) {
                    slidesAngle = robot.intakeSlideMax
                } else {
                    slidesAngle += (.005 * abs(gamepad2.left_stick_y))
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


            if (robot.deliveryLiftDownSwitch.voltage < 2) {
                if (robot.intakeWrist.position != robot.intakeWristLeft) {
                    robot.intakeGripper.position = robot.intakeGripperClearance
                }
            }

            if (gamepad2.right_stick_button) {
                deliveryHeightGoal = robot.specimenDeliveryPosition
            } else {
                deliveryHeightGoal = robot.deliveryMaxHeight
            }

            if (gamepad2.right_stick_y > 0.1 && robot.deliveryLiftDownSwitch.voltage < 2) {
                if (robot.deliveryBack.currentPosition < 100) {
                    robot.deliveryFront.power = 0.0
                    robot.deliveryBack.power = 0.0
                } else {
                    robot.deliveryFront.power = (gamepad2.right_stick_y.toDouble() / 2)
                    robot.deliveryBack.power = (gamepad2.right_stick_y.toDouble() / 2)
                }
            } else
                if (gamepad2.right_stick_y < 0.1 && robot.deliveryBack.currentPosition < deliveryHeightGoal) {
                    if (robot.deliveryBack.currentPosition > deliveryHeightGoal - 100) {
                        robot.deliveryFront.power = -0.1
                        robot.deliveryBack.power = -0.1
                    } else {
                        robot.deliveryFront.power = gamepad2.right_stick_y.toDouble()
                        robot.deliveryBack.power = gamepad2.right_stick_y.toDouble()
                    }
                } else {
                    robot.deliveryFront.power = 0.0
                    robot.deliveryBack.power = 0.0
                }

            if (robot.deliveryLiftDownSwitch.voltage > 2 && !resetEncoder) {
                robot.deliveryBack.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                robot.deliveryBack.mode = DcMotor.RunMode.RUN_USING_ENCODER
                resetEncoder = true
            }

            if (robot.deliveryLiftDownSwitch.voltage < 2) {
                resetEncoder = false
            }

        }
        // Distance Sensors

        //        Action trajectoryActionChosen2;
//        trajectoryActionChosen2 = park.build();
//
//                Actions.runBlocking(
//                new SequentialAction(
//                        trajectoryActionChosen
////                        trajectoryActionChosen2
//
//                )
//        );

//        telemetry.addData("gamepad2 dpad_right", gamepad2.dpad_right)
//        telemetry.addData("Delivery Back Encoder", robot.deliveryBack.currentPosition)
//        telemetry.addData("Action Stage", actionStage)
//        telemetry.addData("runtime", runtime)

//        telemetry.addData("Transfer color sensor", robot.transferDistanceSensor.red())
                telemetry.addData("slides", slidesAngle)
        telemetry.addData("wrist", wristAngle)




        telemetry.update()
    }
}