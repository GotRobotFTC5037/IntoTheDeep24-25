package org.firstinspires.ftc.teamcode

import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.PathChain
import com.pedropathing.pathgen.Point
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.Robot
import kotlin.time.Duration.Companion.seconds


@Autonomous(name = "NewSpecimenAuto", group = "Auto")
class NewSpecimenAuto : OpMode() {

    lateinit var robot: Robot
    private var pathTimer: Timer? = null
    private var sillyTimer: Timer? = null
    private var opmodeTimer: Timer? = null
    private var pathState = 0

    private var sample1Slide = 0.53
    private var sample2Slide = 0.66
    private var sample3Slide = 0.9

    private var sample1Wrist = 0.46
    private var sample2Wrist = 0.35
    private var sample3Wrist = 0.28

    private var sample1and2PivotAngle = 0.65
    private var sample3PivotAngle = 0.65

    private val startPose = Pose(9.0, 55.75, Math.toRadians(0.0))

    private val scorePreload = Pose(37.5, 67.0, Math.toRadians(0.0))

    private val firstSamplePickup = Pose(27.5, 27.25, Math.toRadians(342.0))
    private val firstSamplePickupControlPoint = Pose(27.5, 67.0)
    private val firstSampleDelivery = Pose(26.0, 27.25, Math.toRadians(190.0))

    private val secondSamplePickup = Pose(27.5, 27.25, Math.toRadians(322.0))
    private val secondSampleDelivery = Pose(26.0, 27.25, Math.toRadians(210.0))

    private val thirdSamplePickup = Pose(28.5, 26.25, Math.toRadians(299.0))
    private val thirdSampleDelivery = Pose(26.0, 27.25, Math.toRadians(210.0))

    private val pickupSpecimen1 = Pose(17.5, 8.25, Math.toRadians(270.0))
    private val pickupSpecimen1ControlPoint = Pose(17.5, 27.25)

    private val placeSpecimen1 = Pose(38.0, 71.0, Math.toRadians(0.0))
    private val placeSpecimen1ControlPoint = Pose(17.5, 71.0)

    private val pickupSpecimen2 = Pose(17.0, 8.25, Math.toRadians(270.0))
    private val pickupSpecimen2ControlPoint = Pose(17.0, 27.25)

    private val placeSpecimen2 = Pose(38.0, 73.0, Math.toRadians(0.0))
    private val placeSpecimen2ControlPoint = Pose(17.0, 73.0)

    private val pickupSpecimen3 = Pose(17.0, 8.25, Math.toRadians(270.0))
    private val pickupSpecimen3ControlPoint = Pose(17.0, 27.25)

    private val placeSpecimen3 = Pose(38.0, 75.0, Math.toRadians(0.0))
    private val placeSpecimen3ControlPoint = Pose(17.0, 75.0)

    private val park = Pose(18.0, 15.0, Math.toRadians(0.0))
    private val parkControlPoint = Pose(18.0, 75.0)


    private var action1ScorePreload: PathChain? = null

    private var action2pickupSample1: PathChain? = null
    private var action3placeSample1: PathChain? = null

    private var action4pickupSample2: PathChain? = null
    private var action5placeSample2: PathChain? = null

    private var action6pickupSample3: PathChain? = null
    private var action7placeSample3: PathChain? = null

    private var action8pickupSpecimen1: PathChain? = null
    private var action9scoreSpecimen1: PathChain? = null

    private var action10pickupSpecimen2: PathChain? = null
    private var action11scoreSpecimen2: PathChain? = null

    private var action12pickupSpecimen3: PathChain? = null
    private var action13scoreSpecimen3: PathChain? = null

    private var action14park: PathChain? = null

    private fun buildPaths() {

        action1ScorePreload = robot.follower.pathBuilder()
            .addPath(BezierLine(Point(startPose), Point(scorePreload)))
            .setConstantHeadingInterpolation(scorePreload.heading)
//            .setZeroPowerAccelerationMultiplier(1.0)
            .build()

        action2pickupSample1 = robot.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(scorePreload),
                    Point(firstSamplePickupControlPoint),
                    Point(firstSamplePickup)
                )
            )
            .setConstantHeadingInterpolation(firstSamplePickup.heading)
            .build()

        action3placeSample1 = robot.follower.pathBuilder()
            .addPath(BezierLine(Point(firstSamplePickup), Point(firstSampleDelivery)))
            .setLinearHeadingInterpolation(firstSamplePickup.heading, firstSampleDelivery.heading)
            .build()

        action4pickupSample2 = robot.follower.pathBuilder()
            .addPath(BezierLine(Point(firstSampleDelivery), Point(secondSamplePickup)))
            .setLinearHeadingInterpolation(firstSampleDelivery.heading, secondSamplePickup.heading)
            .build()

        action5placeSample2 = robot.follower.pathBuilder()
            .addPath(BezierLine(Point(secondSamplePickup), Point(secondSampleDelivery)))
            .setLinearHeadingInterpolation(secondSamplePickup.heading, secondSampleDelivery.heading)
            .build()

        action6pickupSample3 = robot.follower.pathBuilder()
            .addPath(BezierLine(Point(secondSampleDelivery), Point(thirdSamplePickup)))
            .setLinearHeadingInterpolation(secondSampleDelivery.heading, thirdSamplePickup.heading)
            .build()

        action7placeSample3 = robot.follower.pathBuilder()
            .addPath(BezierLine(Point(thirdSamplePickup), Point(thirdSampleDelivery)))
            .setLinearHeadingInterpolation(thirdSamplePickup.heading, thirdSampleDelivery.heading)
            .build()

        action8pickupSpecimen1 = robot.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(thirdSampleDelivery),
                    Point(pickupSpecimen1ControlPoint),
                    Point(pickupSpecimen1)
                )
            )
            .setLinearHeadingInterpolation(thirdSampleDelivery.heading, pickupSpecimen1.heading)
            .build()

        action9scoreSpecimen1 = robot.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(pickupSpecimen1),
                    Point(placeSpecimen1ControlPoint),
                    Point(placeSpecimen1)
                )
            )
            .setLinearHeadingInterpolation(pickupSpecimen1.heading, placeSpecimen1.heading)
            .build()

        action10pickupSpecimen2 = robot.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(placeSpecimen1),
                    Point(pickupSpecimen2ControlPoint),
                    Point(pickupSpecimen2)
                )
            )
            .setLinearHeadingInterpolation(placeSpecimen1.heading, pickupSpecimen2.heading)
            .build()

        action11scoreSpecimen2 = robot.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(pickupSpecimen2),
                    Point(placeSpecimen2ControlPoint),
                    Point(placeSpecimen2)
                )
            )
            .setLinearHeadingInterpolation(pickupSpecimen2.heading, placeSpecimen2.heading)
            .build()

        action12pickupSpecimen3 = robot.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(placeSpecimen2),
                    Point(pickupSpecimen3ControlPoint),
                    Point(pickupSpecimen3)
                )
            )
            .setLinearHeadingInterpolation(placeSpecimen2.heading, pickupSpecimen3.heading)
            .build()

        action13scoreSpecimen3 = robot.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(pickupSpecimen3),
                    Point(placeSpecimen3ControlPoint),
                    Point(placeSpecimen3)
                )
            )
            .setLinearHeadingInterpolation(pickupSpecimen3.heading, placeSpecimen3.heading)
            .build()

        action14park = robot.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(placeSpecimen3),
                    Point(parkControlPoint),
                    Point(park)
                )
            )
            .setLinearHeadingInterpolation(placeSpecimen3.heading, park.heading)
            .build()
    }

    private fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                robot.follower.followPath(action1ScorePreload, true)
                robot.follower.setMaxPower(0.7)
                resetRuntime()
                pathState++
            }

            1 -> {
                if (robot.follower.isBusy) {
                    robot.moveLiftToPosition(robot.specimenDeliveryPosition, 0.6)
                } else {
                    robot.moveLiftToBottom()
                    if (robot.deliveryBack.currentPosition < 600) {
                        robot.follower.setMaxPower(1.0)
                        robot.specimenGripper.position = robot.specimenGripperOpen
                        robot.follower.followPath(action2pickupSample1, true)
                        resetRuntime()
                        pathState++
                    }
                }
            }

            2 -> {
                robot.moveLiftToBottom()
//                robot.intakeGripper.position = robot.intakeGripperNeutral
                if (runtime > 1) {
                    robot.intakeSlide.position = sample1Slide
                    robot.intakePivot.position = sample1and2PivotAngle
                    robot.intakeWrist.position = sample1Wrist
                    resetRuntime()
                    pathState++
                }
            }

            3 -> {
                if (!robot.follower.isBusy) {
                    robot.intakeGripper.position = robot.intakeGripperClosedSides
                    resetRuntime()
                    pathState++
                }
            }

            4 -> {
                if (runtime > 0.5) {
                    robot.follower.followPath(action3placeSample1, true)
                    resetRuntime()
                    pathState++
                }
            }

            5 -> {
                if (runtime > 0.75) {
                    robot.intakeGripper.position = robot.intakeGripperNeutral
                    resetRuntime()
                    pathState++
                }
            }

            6 -> {
                if (runtime > 0.1) {
                    robot.follower.followPath(action4pickupSample2, true)
                    robot.intakeSlide.position = sample2Slide
                    robot.intakeWrist.position = sample2Wrist
                    resetRuntime()
                    pathState++
                }
            }

            7 -> {
                if (runtime > 1.25) {
                    robot.intakeGripper.position = robot.intakeGripperClosedSides
                    resetRuntime()
                    pathState++
                }
            }

            8 -> {
                if (runtime > 0.75) {
                    robot.follower.followPath(action5placeSample2, true)
                    resetRuntime()
                    pathState++
                }
            }

            9 -> {
                if (runtime > 0.75) {
                    robot.intakeGripper.position = robot.intakeGripperNeutral
                    resetRuntime()
                    pathState++
                }
            }

            10 -> {
                if (runtime > 0.2) {
                    robot.follower.followPath(action6pickupSample3, true)
                    resetRuntime()
                    pathState++
                }
            }

            11 -> {
                if (runtime > 0.5) {
                    robot.intakeSlide.position = sample3Slide
                    robot.intakeWrist.position = sample3Wrist
                    resetRuntime()
                    pathState++
                }
            }

            12 -> {
                if (runtime > 0.5) {
                    robot.intakeGripper.position = robot.intakeGripperClosedSides
                    resetRuntime()
                    pathState++
                }
            }

            13 -> {
                if (runtime > 0.5) {
                    robot.intakeSlide.position = sample1Slide
                    resetRuntime()
                    pathState++
                }
            }

            14 -> {
                if (runtime > 0.2) {
                    robot.follower.followPath(action7placeSample3, true)
                    resetRuntime()
                    pathState++
                }
            }

            15 -> {
                if (runtime > 0.75) {
                    robot.intakeGripper.position = robot.intakeGripperNeutral
                    robot.intakePivot.position = robot.intakePivotUp
                    robot.intakeWrist.position = robot.intakeWristLeft
                    robot.intakeSlide.position = robot.intakeSlideMin
                    robot.follower.followPath(action8pickupSpecimen1,true)
                    resetRuntime()
                    pathState++
                }
            }

            16 -> {
                if (!robot.follower.isBusy) {
                    robot.specimenGripper.position = robot.specimenGripperClosed
                }
                if (runtime > 1.6) {
                    robot.moveLiftToPosition(robot.specimenDeliveryPosition, 0.5)
                }
                if (robot.deliveryBack.currentPosition > 200) {
                    robot.follower.followPath(action9scoreSpecimen1,true)
                    pathState++
                }
            }

            17 -> {
                if (robot.follower.isBusy) {
                    robot.moveLiftToPosition(robot.specimenDeliveryPosition, 0.5)
                } else {
                    robot.moveLiftToBottom()
                    if (robot.deliveryBack.currentPosition < 600) {
                        robot.specimenGripper.position = robot.specimenGripperOpen
                        robot.follower.followPath(action10pickupSpecimen2, true)
                        pathState++
                    }
                }
            }

            18 -> {
                robot.moveLiftToBottom()
                if (!robot.follower.isBusy) {
                    robot.specimenGripper.position = robot.specimenGripperClosed
                    resetRuntime()
                    pathState++
                }
            }

            19 -> {
                if (runtime > 0.5) {
                    robot.moveLiftToPosition(robot.specimenDeliveryPosition, 0.5)
                }
                if (robot.deliveryBack.currentPosition > 200) {
                    robot.follower.followPath(action11scoreSpecimen2,true)
                    pathState++
                }
            }

            20 -> {
                if (robot.follower.isBusy) {
                    robot.moveLiftToPosition(robot.specimenDeliveryPosition, 0.5)
                } else {
                    robot.moveLiftToBottom()
                    if (robot.deliveryBack.currentPosition < 600) {
                        robot.specimenGripper.position = robot.specimenGripperOpen
                        robot.follower.followPath(action12pickupSpecimen3, true)
                        pathState++
                    }
                }
            }

            21 -> {
                robot.moveLiftToBottom()
                if (!robot.follower.isBusy) {
                    robot.specimenGripper.position = robot.specimenGripperClosed
                    resetRuntime()
                    pathState++
                }
            }

            22 -> {
                if (runtime > 0.5) {
                    robot.moveLiftToPosition(robot.specimenDeliveryPosition, 0.5)
                }
                if (robot.deliveryBack.currentPosition > 200) {
                    robot.follower.followPath(action13scoreSpecimen3,true)
                    pathState++
                }
            }

            23 -> {
                if (robot.follower.isBusy) {
                    robot.moveLiftToPosition(robot.specimenDeliveryPosition, 0.5)
                } else {
                    robot.moveLiftToBottom()
//                    if (robot.deliveryBack.currentPosition < 600) {
//                        robot.specimenGripper.position = robot.specimenGripperOpen
//                        robot.follower.followPath(action14park, true)
//                        pathState++
//                    }
                }
            }


        }
    }

    private fun setPathState(pState: Int) {
        pathState = pState
        pathTimer!!.resetTimer()
    }

    override fun loop() {
        robot.follower.update()
        autonomousPathUpdate()

        telemetry.addData("Path State", pathState)
        telemetry.addData("Runtime", runtime)
        telemetry.addData("x", robot.follower.pose.x)
        telemetry.addData("y", robot.follower.pose.y)
        telemetry.addData("h", robot.follower.pose.heading)
        telemetry.update()

        robot.follower.drawOnDashBoard()
    }

    override fun init() {
        robot = Robot(hardwareMap)

        pathTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer!!.resetTimer()

        robot.follower.setStartingPose(startPose)
        buildPaths()
    }

    override fun init_loop() {
        opmodeTimer!!.resetTimer()
    }

    override fun start() {
        opmodeTimer!!.resetTimer()
        robot.initializeInSpecimen()
        setPathState(0)
    }

    override fun stop() {

    }

}