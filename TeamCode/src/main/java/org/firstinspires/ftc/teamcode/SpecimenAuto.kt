package pedroPathing


import android.os.SystemClock
import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.Path
import com.pedropathing.pathgen.PathChain
import com.pedropathing.pathgen.Point
import com.pedropathing.util.Constants
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Robot
import org.opencv.core.Mat

@Autonomous(name = "SpecimenAuto", group = "Auto")
class SpecimenAuto : OpMode() {
    lateinit var robot: Robot
    private var pathTimer: Timer? = null
    private var sillyTimer: Timer? = null
    private var opmodeTimer: Timer? = null
    private var pathState = 0

    private val startPose = Pose(9.0, 55.75, Math.toRadians(0.0))
    private val scorePreload = Pose(37.75, 67.0, Math.toRadians(0.0))
    private val scorePreloadControlPoint1 = Pose(21.375, 55.75)
    private val scorePreloadControlPoint2 = Pose(21.375, 67.0)
    private val beforeMovingSample1 = Pose(47.0, 33.0, Math.toRadians(0.0))
    private val beforeMovingSample1ControlPoint1 = Pose(20.0, 66.0)
    private val beforeMovingSample1ControlPoint2 = Pose(20.0, 33.0)
    private val beforeMovingSample2 = Pose(55.0, 33.0, Math.toRadians(0.0))
    private val beforeMovingSample3 = Pose(55.0, 27.0, Math.toRadians(0.0))
    private val beforeMovingSample3ControlPoint1 = Pose(60.0, 33.0)
    private val beforeMovingSample3ControlPoint2 = Pose(60.0, 27.0)
    private val pushSample1 = Pose(15.0, 27.0, Math.toRadians(0.0))
    private val comeBackAround = Pose(55.0, 30.0, Math.toRadians(270.0))
    private val comeBackAroundControlPoint1 = Pose(30.0, 30.0, Math.toRadians(270.0))
    private val lineUpForSample2 = Pose(55.0, 16.0, Math.toRadians(270.0))
    private val lineUpForSample2ControlPoint1 = Pose(60.0, 31.0, Math.toRadians(270.0))
    private val lineUpForSample2ControlPoint2 = Pose(60.0, 15.0, Math.toRadians(270.0))
    private val pushSample2InZone = Pose(18.0, 16.0, Math.toRadians(270.0))
    private val pickUpSpecimen = Pose(18.0,9.0, Math.toRadians(270.0))
    private val lineUpForSpecimen1 = Pose(30.0, 73.0, Math.toRadians(0.0))
    private val deliverSpecimen1 = Pose(37.75,73.0, Math.toRadians(0.0))
    private val lineUpForGrab2 = Pose(18.0,73.0, Math.toRadians(270.0))
    private val lineUpForSpecimen2 = Pose(30.0,77.0, Math.toRadians(0.0))
    private val deliverSpecimen2 = Pose(37.75,77.0,Math.toRadians(0.0))
    private val lineUpForGrab3 = Pose(18.0,77.0, Math.toRadians(270.0))
    private val lineUpForSpecimen3 = Pose(30.0,81.0, Math.toRadians(0.0))
    private val deliverSpecimen3 = Pose(37.75,81.0,Math.toRadians(0.0))
    private val lineUpForGrab4 = Pose(18.0,81.0,Math.toRadians(270.0))

    //    private val pickUpSpecimen3= Pose(18.0,9.0, Math.toRadians(270.0))
//    private val deliverSpecimen3 = Pose(37.75,81.0, Math.toRadians(0.0))
//    private val deliverSpecimen3ControlPoint1 = Pose(18.0,81.0)
    private val park = Pose(18.0,15.0, Math.toRadians(0.0))

    private var action1ScorePreload: PathChain? = null
    private var action2AfterPreload: PathChain? = null
    private var action3PushSample2: PathChain? = null
    private var action4PickUpSpecimen1: PathChain? = null
    private var action5LineUpForSpecimen1: PathChain? = null
    private var action6DeliverSpecimen1: PathChain? = null
    private var action7LineUpForGrab2: PathChain? = null
    private var action8GrabSpecimen2: PathChain? = null
    private var action9LineUpForSpecimen2: PathChain? = null
    private var action10DeliverSpecimen2: PathChain? = null
    private var action11LineUpForGrab3: PathChain? = null
    private var action12Park: PathChain? = null
//    private var action12GrabSample3: PathChain? = null
//    private var action13LineUpForSpecimen3: PathChain? = null
//    private var action14DeliverSpecimen3: PathChain? = null
//    private var action15LineUpForGrab4: PathChain? = null
//    private var action16GrabSample4: PathChain? = null
//    private var action15Park: PathChain? = null


    fun buildPaths() {
        action1ScorePreload = robot.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(startPose),
                    Point(scorePreloadControlPoint1),
                    Point(scorePreloadControlPoint2),
                    Point(scorePreload)
                )
            )
            .setConstantHeadingInterpolation(scorePreload.heading)
        .build()


//        action2AfterPreload = robot.follower.pathBuilder()
//            .addPath(BezierLine(Point(firstPosePreOuttake), Point(secondPoseOuttake)))
//            .setLinearHeadingInterpolation(firstPosePreOuttake.heading, secondPoseOuttake.heading)
//            .setZeroPowerAccelerationMultiplier(2.0)
//            .build()
//
        action2AfterPreload = robot.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(scorePreload),
                    Point(beforeMovingSample1ControlPoint1),
                    Point(beforeMovingSample1ControlPoint2),
                    Point(beforeMovingSample1)
                )
            )
            .setConstantHeadingInterpolation(beforeMovingSample1.heading)
            .addPath(BezierLine(Point(beforeMovingSample1), Point(beforeMovingSample2)))
            .setConstantHeadingInterpolation(beforeMovingSample2.heading)
            .addPath(
                BezierCurve(
                    Point(beforeMovingSample2),
                    Point(beforeMovingSample3ControlPoint1),
                    Point(beforeMovingSample3ControlPoint2),
                    Point(beforeMovingSample3)
                )
            )
            .setConstantHeadingInterpolation(beforeMovingSample3.heading)
            .addPath(BezierLine(Point(beforeMovingSample3), Point(pushSample1)))
            .setConstantHeadingInterpolation(pushSample1.heading)
        .build()

        action3PushSample2 = robot.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(pushSample1),
                    Point(comeBackAroundControlPoint1),
                    Point(comeBackAround)
                )
            )
            .setLinearHeadingInterpolation(0.0, comeBackAround.heading)
            .addPath(
                BezierCurve(
                    Point(comeBackAround),
                    Point(lineUpForSample2ControlPoint1),
                    Point(lineUpForSample2ControlPoint2),
                    Point(lineUpForSample2)
                )
            )
            .setConstantHeadingInterpolation(lineUpForSample2.heading)

            .addPath(BezierLine(Point(lineUpForSample2), Point(pushSample2InZone)))
            .setConstantHeadingInterpolation(pushSample2InZone.heading)
        .build()

        action4PickUpSpecimen1 = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(pushSample2InZone),
                    Point(pickUpSpecimen)
                )
            )
            .setConstantHeadingInterpolation(pickUpSpecimen.heading)
        .build()

        action5LineUpForSpecimen1 = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(pickUpSpecimen),
                    Point(lineUpForSpecimen1)
                )
            )
            .setLinearHeadingInterpolation(pickUpSpecimen.heading, lineUpForSpecimen1.heading)
        .build()

        action6DeliverSpecimen1 = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(lineUpForSpecimen1),
                    Point(deliverSpecimen1)
                )
            )
            .setConstantHeadingInterpolation(deliverSpecimen1.heading)
        .build()

        action7LineUpForGrab2 = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(deliverSpecimen1),
                    Point(lineUpForGrab2)
                )
            )
            .setLinearHeadingInterpolation(deliverSpecimen1.heading, lineUpForGrab2.heading)
        .build()

        action8GrabSpecimen2 = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(lineUpForGrab2),
                    Point(pickUpSpecimen)
                )
            )
            .setConstantHeadingInterpolation(pickUpSpecimen.heading)
        .build()

        action9LineUpForSpecimen2 = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(pickUpSpecimen),
                    Point(lineUpForSpecimen2)
                )
            )
            .setLinearHeadingInterpolation(pickUpSpecimen.heading, lineUpForSpecimen2.heading)
        .build()

        action10DeliverSpecimen2 = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(lineUpForSpecimen2),
                    Point(deliverSpecimen2)
                )
            )
            .setConstantHeadingInterpolation(deliverSpecimen2.heading)
        .build()


        action11LineUpForGrab3 = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(deliverSpecimen2),
                    Point(lineUpForGrab3)
                )
            )
            .setLinearHeadingInterpolation(deliverSpecimen2.heading, lineUpForGrab3.heading)
        .build()

        action12Park = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(lineUpForGrab3),
                    Point(park)
                )
            )
            .setConstantHeadingInterpolation(park.heading)
        .build()


//        action12GrabSample3 = robot.follower.pathBuilder()
//            .addPath(
//                BezierLine(
//                    Point(lineUpForGrab3),
//                    Point(pickUpSpecimen)
//                )
//            )
//            .setConstantHeadingInterpolation(pickUpSpecimen.heading)
//        .build()


//        action13LineUpForSpecimen3 = robot.follower.pathBuilder()
//            .addPath(
//                BezierLine(
//                    Point(pickUpSpecimen),
//                    Point(lineUpForSpecimen3)
//                )
//            )
//            .setLinearHeadingInterpolation(pickUpSpecimen.heading, lineUpForSpecimen3.heading)
//        .build()
//
//        action14DeliverSpecimen3 = robot.follower.pathBuilder()
//            .addPath(
//                BezierLine(
//                    Point(lineUpForSpecimen3),
//                    Point(deliverSpecimen3)
//                )
//            )
//            .setConstantHeadingInterpolation(deliverSpecimen3.heading)
//        .build()
    }




    fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                robot.follower.setMaxPower(0.7)
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
                        robot.follower.followPath(action2AfterPreload, true)
                        pathState = 2
                    }
                }
            }

            2 -> {
                robot.moveLiftToBottom()
                if (!robot.follower.isBusy) {
                    robot.follower.setMaxPower(1.0)
                    robot.follower.followPath(action3PushSample2, true)
                    pathState = 3
                }
            }

            3 -> {
                if (!robot.follower.isBusy) {
                    resetRuntime()
                    robot.follower.setMaxPower(0.5)
                    robot.follower.followPath(action4PickUpSpecimen1, true)
                    pathState = 4
                }
            }

            4 -> {
                if (runtime > 1.2) {
                    robot.specimenGripper.position = robot.specimenGripperClosed
                }
                if (runtime > 1.6) {
                    robot.moveLiftToPosition(robot.specimenDeliveryPosition, 0.5)
                }
                if (robot.deliveryBack.currentPosition > 300) {
                    robot.follower.setMaxPower(1.0)
                    robot.follower.followPath(action5LineUpForSpecimen1,true)
                    pathState = 5
                }
            }

            5 -> {
                robot.moveLiftToPosition(robot.specimenDeliveryPosition, 0.6)
                if (!robot.follower.isBusy) {
                    robot.follower.setMaxPower(0.7)
                    robot.follower.followPath(action6DeliverSpecimen1,true)
                    pathState = 6
                }
            }

            6 -> {
                if (robot.follower.isBusy) {
                    robot.moveLiftToPosition(robot.specimenDeliveryPosition, 0.5)
                } else {
                    robot.moveLiftToBottom()
                    if (robot.deliveryBack.currentPosition < 600) {
                        robot.specimenGripper.position = robot.specimenGripperOpen
                        robot.follower.setMaxPower(1.0)
                        robot.follower.followPath(action7LineUpForGrab2, true)
                        pathState = 7
                    }
                }
            }

            7 -> {
                robot.moveLiftToBottom()
                if (!robot.follower.isBusy) {
                    resetRuntime()
                    robot.follower.setMaxPower(1.0)
                    robot.follower.followPath(action8GrabSpecimen2, true)
                    pathState = 8
                }
            }

            8 -> {
                if (runtime > 1.8) {
                    robot.specimenGripper.position = robot.specimenGripperClosed
                }
                if (runtime > 2) {
                    robot.moveLiftToPosition(robot.specimenDeliveryPosition, 0.5)
                }
                if (robot.deliveryBack.currentPosition > 300) {
                    robot.follower.setMaxPower(1.0)
                    robot.follower.followPath(action9LineUpForSpecimen2,true)
                    pathState = 9
                }
            }

            9 -> {
                robot.moveLiftToPosition(robot.specimenDeliveryPosition, 0.5)
                if (!robot.follower.isBusy) {
                    robot.follower.setMaxPower(0.7)
                    robot.follower.followPath(action10DeliverSpecimen2,true)
                    pathState = 10
                }
            }

            10 -> {
                if (robot.follower.isBusy) {
                    robot.moveLiftToPosition(robot.specimenDeliveryPosition, 0.5)
                } else {
                    robot.moveLiftToBottom()
                    if (robot.deliveryBack.currentPosition < 600) {
                        robot.specimenGripper.position = robot.specimenGripperOpen
                        robot.follower.setMaxPower(1.0)
                        robot.follower.followPath(action11LineUpForGrab3, true)
                        pathState = 11
                    }
                }
            }

            11 -> {
                robot.moveLiftToBottom()
                if (!robot.follower.isBusy) {
                    resetRuntime()
                    robot.follower.setMaxPower(1.0)
                    robot.follower.followPath(action12GrabSample3, true)
                    pathState = 12
                }
            }

            13 -> {

            }

//            12 -> {
//                if (runtime > 1.8) {
//                    robot.specimenGripper.position = robot.specimenGripperClosed
//                }
//                if (runtime > 2) {
//                    robot.moveLiftToPosition(robot.specimenDeliveryPosition, 0.5)
//                }
//                if (robot.deliveryBack.currentPosition > 300) {
//                    robot.follower.setMaxPower(1.0)
//                    robot.follower.followPath(action13LineUpForSpecimen3,true)
//                    pathState = 13
//                }
//            }
//
//            13 -> {
//                robot.moveLiftToPosition(robot.specimenDeliveryPosition, 0.5)
//                if (!robot.follower.isBusy) {
//                    robot.follower.setMaxPower(0.7)
//                    robot.follower.followPath(action14DeliverSpecimen3,true)
//                    pathState = 14
//                }
//            }
//
//            14 -> {
//                if (robot.follower.isBusy) {
//                    robot.moveLiftToPosition(robot.specimenDeliveryPosition, 0.5)
//                } else {
//                    robot.moveLiftToBottom()
//                    if (robot.deliveryBack.currentPosition < 600) {
//                        robot.specimenGripper.position = robot.specimenGripperOpen
//                        robot.follower.setMaxPower(1.0)
//                        robot.follower.followPath(action15LineUpForGrab4, true)
//                        pathState = 15
//                    }
//                }
//            }
//
//            15 -> {
//                robot.moveLiftToBottom()
//                if (!robot.follower.isBusy) {
//                    robot.follower.setMaxPower(1.0)
//                    robot.follower.followPath(action16GrabSample4, true)
//                    pathState = 16
//                }
//            }
//
//            16 -> {
//                if (!robot.follower.isBusy) {
//                    pathState = -1
//                }
//            }

//            2 -> if (!robot.follower.isBusy) {
//                robot.follower.setMaxPower(1.0)
//                robot.follower.followPath(action2OuttakeOne, true)
//                setPathState(3)
//            }
//
//            3 -> if (!robot.follower.isBusy) {
//                robot.follower.holdPoint(secondPoseOuttake)
//                SystemClock.sleep(450)
//                robot.follower.setMaxPower(1.0)
//                robot.follower.followPath(action3DoubleCurvedPrePush, true)
//                setPathState(4)
//            }
//
//            4 -> if (!robot.follower.isBusy) {
//                robot.follower.followPath(action4PushOne, true)
//                setPathState(5)
//            }
//
//            5 -> if (!robot.follower.isBusy) {
//                robot.follower.followPath(action5CurvedForPushTwo, true)
//                setPathState(6)
//            }
//
//            6 -> if (!robot.follower.isBusy) {
//                robot.follower.followPath(action6PushTwo, true)
//                setPathState(7)
//            }
//
//            7 -> if (!robot.follower.isBusy) {
//                robot.follower.followPath(action9PreIntakeOne, true)
//                setPathState(-1)
//            }
        }
    }

    fun setPathState(pState: Int) {
        pathState = pState
        pathTimer!!.resetTimer()
    }

    override fun loop() {
        robot.follower.update()
        autonomousPathUpdate()
        telemetry.addData("runtime", runtime)
        telemetry.addData("path state", pathState)
        telemetry.addData("x", robot.follower.pose.x)
        telemetry.addData("y", robot.follower.pose.y)
        telemetry.addData("heading", robot.follower.pose.heading)
        telemetry.addData("lift", robot.deliveryBack.currentPosition)
        telemetry.update()
        robot.follower.drawOnDashBoard()
    }


    override fun init() {
        robot = Robot(hardwareMap)
        pathTimer = Timer()
        opmodeTimer = Timer()
        sillyTimer = Timer()
        opmodeTimer!!.resetTimer()
        robot.follower.setStartingPose(startPose)
        buildPaths()
    }

    override fun init_loop() {
        opmodeTimer!!.resetTimer()
        sillyTimer!!.resetTimer()
    }

    override fun start() {
        sillyTimer!!.resetTimer()
        opmodeTimer!!.resetTimer()
        robot.initializeInSpecimen()
        setPathState(0)
    }

    override fun stop() {
    }
}