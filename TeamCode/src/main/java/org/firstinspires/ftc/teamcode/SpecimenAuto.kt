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
    private val pushSample2InZone = Pose(15.0, 16.0, Math.toRadians(270.0))
    private val pickUpWithSpecimen = Pose(18.0,9.0, Math.toRadians(270.0))
    private val pickUpWithSpecimenControlPoint1 = Pose(20.0,16.0)

    private var action1ScorePreload: PathChain? = null
    private var action2AfterPreload: PathChain? = null
    private var action3PushSample1: PathChain? = null
    private var action4PushSample2: PathChain? = null
    private var action5PickUpSpecimen1: PathChain? = null

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
            .build()

        action3PushSample1 = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(beforeMovingSample3),
                    Point(pushSample1)
                )
            )
            .setConstantHeadingInterpolation(pushSample1.heading)

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

            .build()

        action4PushSample2 = robot.follower.pathBuilder()
            .addPath(
                BezierLine (
                    Point(lineUpForSample2),
                    Point(pushSample2InZone)
                )
            )
            .build()

        action5PickUpSpecimen1 = robot.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(pushSample2InZone),
                    Point(pickUpWithSpecimenControlPoint1),
                    Point(pickUpWithSpecimen)
                )
            )
            .setConstantHeadingInterpolation(pickUpWithSpecimen.heading)
            .build()

    }

    fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                robot.follower.setMaxPower(0.5)
                robot.follower.followPath(action1ScorePreload, true)
                pathState = 1
            }
            1 -> {
                if (robot.follower.isBusy) {
                    robot.moveLiftToPosition(1400, 0.5)
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
                    robot.follower.followPath(action3PushSample1, true)
                    pathState = 3
                }
            }

            3 -> {
                if (!robot.follower.isBusy) {
                    robot.follower.setMaxPower(1.0)
                    robot.follower.followPath(action4PushSample2, true)
                }
            }

            4 -> {
                if (!robot.follower.isBusy) {
                    robot.follower.setMaxPower(0.3)
                    robot.follower.followPath(action5PickUpSpecimen1, true)
                    pathState = 4
                }
            }

            5 -> {
                if (!robot.follower.isBusy) {
                    pathState = -1
                }
            }

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
        robot.initializeInAuto()
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
        setPathState(0)
    }

    override fun stop() {
    }
}