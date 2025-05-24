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

@Autonomous(name = "BasketAuto", group = "Auto")
class BasketAuto : OpMode() {
    lateinit var robot: Robot
    private var pathTimer: Timer? = null
    private var sillyTimer: Timer? = null
    private var opmodeTimer: Timer? = null
    private var pathState = 0
    private var actionStage = 0
    private var sadAngle = 4

    private val startPose = Pose(8.5, 9.0, Math.toRadians(90.0))
    private val scoreSample = Pose(15.5, 33.1, Math.toRadians(130.0))
    private val grabRightSample = Pose(20.0, 32.5, Math.toRadians(335.0 + sadAngle))
    private val grabMidSample = Pose(20.0, 32.5, Math.toRadians(0.0 + sadAngle))
    private val grabLeftSample = Pose(20.5, 32.5, Math.toRadians(17.0 + sadAngle))
    private val park = Pose(18.3, 30.0, Math.toRadians(135.0))


    private var action1scorePreload: PathChain? = null
    private var action2GrabLeft: PathChain? = null
    private var action3Deliver2: PathChain? = null
    private var action4GrabRight: PathChain? = null
    private var action5Deliver3: PathChain? = null
    private var action6GrabMid: PathChain? = null
    private var action7Deliver4: PathChain? = null
    private var action8Park: PathChain? = null




    fun buildPaths() {

        action1scorePreload = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(startPose),
                    Point(scoreSample)
                )
            )
            .setLinearHeadingInterpolation(startPose.heading, scoreSample.heading)
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

        action8Park = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(scoreSample),
                    Point(park)
                )
            )
            .setConstantHeadingInterpolation(park.heading)
            .build()

    }




    fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                robot.follower.setMaxPower(0.8)
                pathState++
            }

            1 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.6)
                if (!robot.follower.isBusy) {
                    robot.follower.followPath(action1scorePreload, true)
                    pathState++
                }
            }

            2 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.5)
                if (!robot.follower.isBusy && robot.deliveryBack.currentPosition > 2450) {
                    robot.deliveryPivot.position = robot.deliveryPivotMedium
                    resetRuntime()
                    pathState++
                }
            }

            3 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.5)
                if(runtime > 0.3) {
                    robot.deliveryGripper.position = robot.deliveryGripperOpen
                    resetRuntime()
                    pathState++
                }
            }

            4 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.5)
                if(runtime > 0.2) {
                    robot.deliveryPivot.position = robot.deliveryPivotHigh
                    resetRuntime()
                    pathState++
                }
            }

            5 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.5)
                if (runtime > 0.3) {
                    robot.follower.followPath(action2GrabLeft, true)
                    pathState++
                }
            }

            6 -> {
                robot.moveLiftToBottom()
                if (!robot.follower.isBusy) {
                    resetRuntime()
                    robot.intakeSlide.position = robot.intakeSlideAutoLeft
                    pathState++
                }
            }

            7 -> {
                if (runtime > 0.8) {
                    resetRuntime()
                    robot.intakePivot.position = robot.intakePivotDown
                    robot.intakeWrist.position = robot.intakeWristAutoLeft
                    pathState++
                }
            }

            8 -> {
                if (runtime > 0.8) {
                    resetRuntime()
                    robot.intakeGripper.position = robot.intakeGripperClosedLoose
                    pathState++
                }
            }

            9 -> {
                if (runtime > 0.4) {
                    intakeAutomation()
                }
                if (actionStage == 6) {
                    robot.deliveryPivot.position = robot.deliveryPivotHigh
                    resetRuntime()
                    actionStage = 0
                    pathState++
                }
            }

            10 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.6)
                if (runtime > 0.6) {
                    robot.follower.followPath(action3Deliver2, true)
                    pathState++
                }
            }

            11 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.5)
                if (!robot.follower.isBusy && robot.deliveryBack.currentPosition > 2450) {
                    robot.deliveryPivot.position = robot.deliveryPivotMedium
                    resetRuntime()
                    pathState++
                }
            }

            12 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.5)
                if(runtime > 0.3) {
                    robot.deliveryGripper.position = robot.deliveryGripperOpen
                    resetRuntime()
                    pathState++
                }
            }

            13 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.5)
                if(runtime > 0.2) {
                    robot.deliveryPivot.position = robot.deliveryPivotHigh
                    resetRuntime()
                    pathState++
                }
            }

            14 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.5)
                if (runtime > 0.3) {
                    robot.follower.followPath(action4GrabRight, true)
                    pathState++
                }
            }

            15 -> {
                robot.moveLiftToBottom()
                if (!robot.follower.isBusy) {
                    resetRuntime()
                    robot.intakeSlide.position = robot.intakeSlideAutoRight
                    pathState++
                }
            }

            16 -> {
                if (runtime > 0.8) {
                    resetRuntime()
                    robot.intakePivot.position = robot.intakePivotDown
                    robot.intakeWrist.position = robot.intakeWristAutoRight
                    robot.intakeGripper.position = robot.intakeGripperNeutral
                    pathState++
                }
            }

            17 -> {
                if (runtime > 0.8) {
                    resetRuntime()
                    robot.intakeGripper.position = robot.intakeGripperClosedLoose
                    pathState++
                }
            }

            18 -> {
                if (runtime > 0.4) {
                    intakeAutomation()
                }
                if (actionStage == 6) {
                    robot.deliveryPivot.position = robot.deliveryPivotHigh
                    resetRuntime()
                    actionStage = 0
                    pathState++
                }
            }

            19 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.6)
                if (runtime > 0.6) {
                    robot.follower.followPath(action5Deliver3, true)
                    pathState++
                }
            }

            20 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.5)
                if (!robot.follower.isBusy && robot.deliveryBack.currentPosition > 2450) {
                    robot.deliveryPivot.position = robot.deliveryPivotMedium
                    resetRuntime()
                    pathState++
                }
            }

            21 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.5)
                if(runtime > 0.3) {
                    robot.deliveryGripper.position = robot.deliveryGripperOpen
                    resetRuntime()
                    pathState++
                }
            }

            22 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.5)
                if(runtime > 0.2) {
                    robot.deliveryPivot.position = robot.deliveryPivotHigh
                    resetRuntime()
                    pathState++
                }
            }

            23 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.5)
                if (runtime > 0.3) {
                    robot.follower.followPath(action6GrabMid, true)
                    pathState++
                }
            }

            24 -> {
                robot.moveLiftToBottom()
                if (!robot.follower.isBusy) {
                    resetRuntime()
                    robot.intakeSlide.position = robot.intakeSlideAutoMid
                    pathState++
                }
            }

            25 -> {
                if (runtime > 0.8) {
                    resetRuntime()
                    robot.intakePivot.position = robot.intakePivotDown
                    robot.intakeWrist.position = robot.intakeWristAutoMid
                    robot.intakeGripper.position = robot.intakeGripperNeutral
                    pathState++
                }
            }

            26 -> {
                if (runtime > 0.8) {
                    resetRuntime()
                    robot.intakeGripper.position = robot.intakeGripperClosedLoose
                    pathState++
                }
            }

            27 -> {
                if (runtime > 0.4) {
                    intakeAutomation()
                }
                if (actionStage == 6) {
                    robot.deliveryPivot.position = robot.deliveryPivotHigh
                    resetRuntime()
                    actionStage = 0
                    pathState++
                }
            }

            28 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.6)
                if (runtime > 0.6) {
                    robot.follower.followPath(action7Deliver4, true)
                    pathState++
                }
            }

            29 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.5)
                if (!robot.follower.isBusy && robot.deliveryBack.currentPosition > 2450) {
                    robot.deliveryPivot.position = robot.deliveryPivotMedium
                    resetRuntime()
                    pathState++
                }
            }

            30 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.5)
                if(runtime > 0.3) {
                    robot.deliveryGripper.position = robot.deliveryGripperOpen
                    resetRuntime()
                    pathState++
                }
            }

            31 -> {
                robot.moveLiftToPosition(robot.deliveryMaxHeight, 0.5)
                if(runtime > 0.2) {
                    robot.deliveryPivot.position = robot.deliveryPivotHigh
                    resetRuntime()
                    pathState++
                }
            }

            32 -> {
                if (runtime > 0.3) {
                    robot.moveLiftToBottom()
                    robot.follower.followPath(action8Park)
                    pathState = -1
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

    fun intakeAutomation() {
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
                if (runtime >= .3) {
                    robot.intakeSlide.position = robot.intakeSlideMin
                    resetRuntime()
                    actionStage++
                }
            }

            4 -> {
                if (runtime >= .35) {
                    robot.deliveryGripper.position = robot.deliveryGripperClosed
                    resetRuntime()
                    actionStage++
                }
            }

            5 -> {
                if (runtime >= .15) {
                    robot.intakeGripper.position = robot.intakeGripperClearance
                    actionStage = 6
                }
            }


        }
    }
}