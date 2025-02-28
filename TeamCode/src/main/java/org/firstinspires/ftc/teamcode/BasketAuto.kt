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

@Autonomous(name = "BacketAuto", group = "Auto")
class BasketAuto : OpMode() {
    lateinit var robot: Robot
    private var pathTimer: Timer? = null
    private var sillyTimer: Timer? = null
    private var opmodeTimer: Timer? = null
    private var pathState = 0

    private val startPose = Pose(8.5, 9.0, Math.toRadians(90.0))
    private val leaveStartPose = Pose(15.0, 9.0, Math.toRadians(90.0))
    private val scoreSample = Pose(14.5, 32.5, Math.toRadians(135.0))
    private val grabRightSample = Pose(20.0, 32.5, Math.toRadians(345.0))
    private val grabMidSample = Pose(20.0, 32.5, Math.toRadians(5.0))
    private val grabLeftSample = Pose(20.0, 32.5, Math.toRadians(20.0))


    private var action1scorePreload: PathChain? = null
    private var action0moveFromWall: PathChain? = null
    private var action2GrabLeft: PathChain? = null
    private var action3Deliver2: PathChain? = null
    private var action4GrabRight: PathChain? = null
    private var action5Deliver3: PathChain? = null
    private var action6GrabMid: PathChain? = null
    private var action7Deliver4: PathChain? = null




    fun buildPaths() {
        action0moveFromWall = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(startPose),
                    Point(leaveStartPose)
                )
            )
            .setConstantHeadingInterpolation(leaveStartPose.heading)
            .build()

        action1scorePreload = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(leaveStartPose),
                    Point(scoreSample)
                )
            )
            .setLinearHeadingInterpolation(leaveStartPose.heading, scoreSample.heading)
        .build()

        action2GrabLeft = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(scoreSample),
                    Point(grabLeftSample)
                )
            )
            .setLinearHeadingInterpolation(scoreSample.heading, grabLeftSample.heading)
        .build()

        action3Deliver2 = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(grabLeftSample),
                    Point(scoreSample)
                )
            )
            .setLinearHeadingInterpolation(grabLeftSample.heading, scoreSample.heading)
            .build()

        action4GrabRight = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(scoreSample),
                    Point(grabRightSample)
                )
            )
            .setLinearHeadingInterpolation(scoreSample.heading, grabRightSample.heading)
            .build()

        action5Deliver3 = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(grabRightSample),
                    Point(scoreSample)
                )
            )
            .setLinearHeadingInterpolation(grabRightSample.heading, scoreSample.heading)
            .build()

        action6GrabMid = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(scoreSample),
                    Point(grabMidSample)
                )
            )
            .setLinearHeadingInterpolation(scoreSample.heading, grabMidSample.heading)
            .build()

        action7Deliver4 = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(grabMidSample),
                    Point(scoreSample)
                )
            )
            .setLinearHeadingInterpolation(grabMidSample.heading, scoreSample.heading)
            .build()

    }




    fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                robot.follower.setMaxPower(0.5)
                robot.follower.followPath(action0moveFromWall, true)
                pathState = 1
            }

            1 -> {
                if (!robot.follower.isBusy) {
                    robot.follower.setMaxPower(0.5)
                    robot.follower.followPath(action1scorePreload, true)
                    pathState = 2
                }
            }

            2 -> {
                if (!robot.follower.isBusy) {
                    pathState = -1
                }
            }

            1 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.5)
                if (!robot.follower.isBusy && robot.deliveryBack.currentPosition > 2450) {
                    robot.deliveryPivot.position = robot.deliveryPivotMedium
                    resetRuntime()
                    pathState = 2
                }
            }

            2 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.5)
                if(runtime > 0.5) {
                    robot.deliveryGripper.position = robot.deliveryGripperOpen
                    resetRuntime()
                    pathState = 3
                }
            }

            3 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.5)
                if(runtime > 0.3) {
                    robot.deliveryPivot.position = robot.deliveryPivotHigh
                    resetRuntime()
                    pathState = 4
                }
            }

            4 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.5)
                if (runtime > 0.5) {

                }
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
        robot.initializeInBasket()
        setPathState(0)
    }

    override fun stop() {
    }
}