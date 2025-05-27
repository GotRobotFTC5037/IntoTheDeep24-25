package org.firstinspires.ftc.teamcode

import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.PathChain
import com.pedropathing.pathgen.Point
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode

@Autonomous(name = "NewSpecimenAuto", group = "Auto")
class NewSpecimenAuto : OpMode() {

    lateinit var robot: Robot
    private var pathTimer: Timer? = null
    private var sillyTimer: Timer? = null
    private var opmodeTimer: Timer? = null
    private var pathState = 0

    private val startPose = Pose(9.0, 55.75, Math.toRadians(0.0))

    private val scorePreload = Pose(38.0, 67.0, Math.toRadians(0.0))

    private val firstSamplePickup = Pose(27.5, 27.25, Math.toRadians(350.0))
    private val firstSamplePickupControlPoint = Pose(27.5, 67.0)
    private val firstSampleDelivery = Pose(27.5, 27.25, Math.toRadians(200.0))

    private val secondSamplePickup = Pose(27.5, 27.25, Math.toRadians(320.0))
    private val secondSampleDelivery = Pose(27.5, 27.25, Math.toRadians(210.0))

    private val thirdSamplePickup = Pose(27.5, 27.25, Math.toRadians(300.0))
    private val thirdSampleDelivery = Pose(27.5, 27.25, Math.toRadians(210.0))

    private val pickupSpecimen1 = Pose(18.0, 9.0, Math.toRadians(270.0))
    private val pickupSpecimen1ControlPoint = Pose(18.0, 27.25)

    private val placeSpecimen1 = Pose(38.0, 71.0, Math.toRadians(0.0))
    private val placeSpecimen1ControlPoint = Pose(18.0, 71.0)

    private val pickupSpecimen2 = Pose(18.0, 9.0, Math.toRadians(270.0))
    private val pickupSpecimen2ControlPoint = Pose(18.0, 27.25)

    private val placeSpecimen2 = Pose(38.0, 73.0, Math.toRadians(0.0))
    private val placeSpecimen2ControlPoint = Pose(18.0, 71.0)

    private val pickupSpecimen3 = Pose(18.0, 9.0, Math.toRadians(270.0))
    private val pickupSpecimen3ControlPoint = Pose(18.0, 27.25)

    private val placeSpecimen3 = Pose(38.0, 75.0, Math.toRadians(0.0))
    private val placeSpecimen3ControlPoint = Pose(18.0, 71.0)

    private val park = Pose(18.0, 15.0, Math.toRadians(0.0))
    private val parkControlPoint = Pose(15.0, 71.0)


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

    fun buildPaths() {
        action1ScorePreload = robot.follower.pathBuilder()
            .addPath(BezierLine(Point(startPose), Point(scorePreload)))
            .setLinearHeadingInterpolation(startPose.heading, scorePreload.heading)
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
            .setLinearHeadingInterpolation(firstSamplePickup.heading, firstSampleDelivery.heading)
            .build()

        action4pickupSample2 = robot.follower.pathBuilder()
            .setLinearHeadingInterpolation(firstSampleDelivery.heading, secondSamplePickup.heading)
            .build()

        action5placeSample2 = robot.follower.pathBuilder()
            .setLinearHeadingInterpolation(secondSamplePickup.heading, secondSampleDelivery.heading)
            .build()

        action6pickupSample3 = robot.follower.pathBuilder()
            .setLinearHeadingInterpolation(secondSampleDelivery.heading, thirdSamplePickup.heading)
            .build()

        action7placeSample3 = robot.follower.pathBuilder()
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

    fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
//                robot.follower.setMaxPower(0.7)
                robot.follower.followPath(action1ScorePreload, true)
                pathState = 1
            }

            1 -> {
                if (robot.follower.isBusy) {
                    robot.moveLiftToPosition(robot.specimenDeliveryPosition, 0.6)
                } else {
                    robot.moveLiftToBottom()
                    if (robot.deliveryBack.currentPosition < 600) {
                        robot.specimenGripper.position = robot.specimenGripperOpen
                        robot.follower.setMaxPower(1.0)
                        robot.follower.followPath(action2pickupSample1, true)
                        pathState = 2
                    }
                }
            }

            2 -> {
                robot.moveLiftToBottom()
                if (!robot.follower.isBusy) {
                    robot.follower.setMaxPower(0.7)
                    robot.follower.followPath(action3placeSample1, true)
                    pathState = 3
                }
            }
        }
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
        pathState = 0
    }

    override fun loop() {
        robot.follower.update()
        autonomousPathUpdate()

        telemetry.addData("Path State", pathState)
        telemetry.addData("x", robot.follower.pose.x)
        telemetry.addData("y", robot.follower.pose.y)
        telemetry.addData("h", robot.follower.pose.heading)
        telemetry.update()

        robot.follower.drawOnDashBoard()
    }

    override fun stop() {

    }

}