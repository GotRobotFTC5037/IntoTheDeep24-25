package pedroPathing


import android.os.SystemClock
import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.PathChain
import com.pedropathing.pathgen.Point
import com.pedropathing.util.Constants
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Robot

@Autonomous(name = "SpecimenAuto", group = "testing")
class SpecimenAuto : OpMode() {
    lateinit var robot: Robot
    private val telemetryA: Telemetry? = null
    private var pathTimer: Timer? = null
    private var sillyTimer: Timer? = null
    private val actionTimer: Timer? = null
    private var opmodeTimer: Timer? = null
    private var pathState = 0
    private val startPose = Pose(135.0, 80.4, Math.toRadians(180.0))
    private val firstPosePreOuttake = Pose(110.0, 78.0, Math.toRadians(180.0))
    private val secondPoseOuttake = Pose(106.5, 78.0, Math.toRadians(180.0))
    private val thirdPosePrePushControlOne = Pose(132.0, 118.0, Math.toRadians(180.0))
    private val thirdPosePrePushControlTwo = Pose(79.0, 103.0, Math.toRadians(180.0))
    private val thirdPosePrePushOne = Pose(85.0, 119.0, Math.toRadians(180.0))
    private val fourthPosePostPushOne = Pose(128.0, 119.0, Math.toRadians(180.0))
    private val fifthPosePrePushTwoControlOne = Pose(81.0, 108.0, Math.toRadians(90.0))
    private val fifthPosePrePushTwo = Pose(84.0, 130.0, Math.toRadians(90.0))
    private val sixthPosePostPushTwo = Pose(128.0, 130.0, Math.toRadians(90.0))
    private val ninthPosePreIntakeOne = Pose(126.0, 135.0, Math.toRadians(90.0))
    private var action6PushTwo: PathChain? = null
    private var action9PreIntakeOne: PathChain? = null
    private var action1PreOuttakeOne: PathChain? = null
    private var action5CurvedForPushTwo: PathChain? = null
    private var action2OuttakeOne: PathChain? = null
    private var action3DoubleCurvedPrePush: PathChain? = null
    private var action4PushOne: PathChain? = null

    fun buildPaths() {
        action1PreOuttakeOne = robot.follower.pathBuilder()
            .addPath(BezierLine(Point(startPose), Point(firstPosePreOuttake)))
            .setLinearHeadingInterpolation(startPose.heading, firstPosePreOuttake.heading)
            .build()

        action2OuttakeOne = robot.follower.pathBuilder()
            .addPath(BezierLine(Point(firstPosePreOuttake), Point(secondPoseOuttake)))
            .setLinearHeadingInterpolation(firstPosePreOuttake.heading, secondPoseOuttake.heading)
            .setZeroPowerAccelerationMultiplier(2.0)
            .build()

        action3DoubleCurvedPrePush = robot.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(secondPoseOuttake),
                    Point(thirdPosePrePushControlOne),
                    Point(thirdPosePrePushControlTwo),
                    Point(thirdPosePrePushOne)
                )
            )
            .setLinearHeadingInterpolation(secondPoseOuttake.heading, thirdPosePrePushOne.heading)
            .build()

        action4PushOne = robot.follower.pathBuilder()
            .addPath(BezierLine(Point(thirdPosePrePushOne), Point(fourthPosePostPushOne)))
            .setLinearHeadingInterpolation(
                thirdPosePrePushOne.heading,
                fourthPosePostPushOne.heading
            )
            .setZeroPowerAccelerationMultiplier(5.0)
            .build()

        action5CurvedForPushTwo = robot.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(fourthPosePostPushOne),
                    Point(fifthPosePrePushTwoControlOne),
                    Point(fifthPosePrePushTwo)
                )
            )
            .setLinearHeadingInterpolation(
                fourthPosePostPushOne.heading,
                fifthPosePrePushTwo.heading
            )
            .build()

        action6PushTwo = robot.follower.pathBuilder()
            .addPath(BezierLine(Point(fifthPosePrePushTwo), Point(sixthPosePostPushTwo)))
            .setLinearHeadingInterpolation(
                fifthPosePrePushTwo.heading,
                sixthPosePostPushTwo.heading
            )
            .setZeroPowerAccelerationMultiplier(5.0)
            .build()

        action9PreIntakeOne = robot.follower.pathBuilder()
            .addPath(BezierLine(Point(sixthPosePostPushTwo), Point(ninthPosePreIntakeOne)))
            .setLinearHeadingInterpolation(
                sixthPosePostPushTwo.heading,
                ninthPosePreIntakeOne.heading
            )
            .build()
    }

    fun autonomousPathUpdate() {
        when (pathState) {
            0 -> setPathState(1)
            1 -> if (!robot.follower.isBusy) {
                robot.follower.setMaxPower(1.0)
                robot.follower.followPath(action1PreOuttakeOne, true)
                setPathState(2)
            }

            2 -> if (!robot.follower.isBusy) {
                robot.follower.setMaxPower(1.0)
                robot.follower.followPath(action2OuttakeOne, true)
                setPathState(3)
            }

            3 -> if (!robot.follower.isBusy) {
                robot.follower.holdPoint(secondPoseOuttake)
                SystemClock.sleep(450)
                robot.follower.setMaxPower(1.0)
                robot.follower.followPath(action3DoubleCurvedPrePush, true)
                setPathState(4)
            }

            4 -> if (!robot.follower.isBusy) {
                robot.follower.followPath(action4PushOne, true)
                setPathState(5)
            }

            5 -> if (!robot.follower.isBusy) {
                robot.follower.followPath(action5CurvedForPushTwo, true)
                setPathState(6)
            }

            6 -> if (!robot.follower.isBusy) {
                robot.follower.followPath(action6PushTwo, true)
                setPathState(7)
            }

            7 -> if (!robot.follower.isBusy) {
                robot.follower.followPath(action9PreIntakeOne, true)
                setPathState(-1)
            }
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