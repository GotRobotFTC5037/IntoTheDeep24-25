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

@Autonomous(name = "SpecimenAuto", group = "Auto")
class SpecimenAuto : OpMode() {
    lateinit var robot: Robot
    private var pathTimer: Timer? = null
    private var sillyTimer: Timer? = null
    private var opmodeTimer: Timer? = null
    private var pathState = 0

    private val startPose = Pose(7.0, 55.75, Math.toRadians(180.0))
    private val toBarPose = Pose(37.7, 67.0, Math.toRadians(180.0))

    private var action1ToBar: PathChain? = null

    fun buildPaths() {
        action1ToBar = robot.follower.pathBuilder()
            .addPath(BezierLine(Point(startPose), Point(toBarPose)))
            .setLinearHeadingInterpolation(startPose.heading, toBarPose.heading)
            .build()

//        action2OuttakeOne = robot.follower.pathBuilder()
//            .addPath(BezierLine(Point(firstPosePreOuttake), Point(secondPoseOuttake)))
//            .setLinearHeadingInterpolation(firstPosePreOuttake.heading, secondPoseOuttake.heading)
//            .setZeroPowerAccelerationMultiplier(2.0)
//            .build()
//
//        action3DoubleCurvedPrePush = robot.follower.pathBuilder()
//            .addPath(
//                BezierCurve(
//                    Point(secondPoseOuttake),
//                    Point(thirdPosePrePushControlOne),
//                    Point(thirdPosePrePushControlTwo),
//                    Point(thirdPosePrePushOne)
//                )
//            )
//            .setLinearHeadingInterpolation(secondPoseOuttake.heading, thirdPosePrePushOne.heading)
//            .build()
    }

    fun autonomousPathUpdate() {
        when (pathState) {
            0 -> setPathState(1)
            1 -> if (!robot.follower.isBusy) {
                robot.follower.setMaxPower(1.0)
                robot.follower.followPath(action1ToBar, true)
                setPathState(2)
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