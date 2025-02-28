package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import com.pedropathing.follower.FollowerConstants
import com.pedropathing.localization.Encoder
import com.pedropathing.localization.Localizers
import com.pedropathing.localization.constants.ThreeWheelConstants
import com.pedropathing.util.Constants
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.abs
import kotlin.math.max


class Robot(val hardwareMap: HardwareMap) {

    object RobotFollowerConstants {
        init {
            FollowerConstants.localizers = Localizers.THREE_WHEEL

            FollowerConstants.leftFrontMotorName = "fl"
            FollowerConstants.leftRearMotorName = "bl"
            FollowerConstants.rightFrontMotorName = "fr"
            FollowerConstants.rightRearMotorName = "br"

            FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE
            FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.FORWARD
            FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE
            FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.REVERSE

            FollowerConstants.mass = 16.56

            FollowerConstants.xMovement = 49.336
            FollowerConstants.yMovement = 35.4184

            FollowerConstants.forwardZeroPowerAcceleration = -38.6004
            FollowerConstants.lateralZeroPowerAcceleration = -84.3702

            FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.2, 0.0, 0.025, 0.0)
            FollowerConstants.useSecondaryTranslationalPID = false
            FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(
                0.1,
                0.0,
                0.01,
                0.0
            ) // Not being used, @see useSecondaryTranslationalPID

            FollowerConstants.headingPIDFCoefficients.setCoefficients(3.0, 0.0, 0.04, 0.0)
            FollowerConstants.useSecondaryHeadingPID = false
            FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(
                2.0,
                0.0,
                0.1,
                0.0
            ) // Not being used, @see useSecondaryHeadingPID

            FollowerConstants.drivePIDFCoefficients.setCoefficients(0.01, 0.0, 0.000008, 0.6, 0.0)
            FollowerConstants.useSecondaryDrivePID = false
            FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(
                0.1,
                0.0,
                0.0,
                0.6,
                0.0
            ) // Not being used, @see useSecondaryDrivePID

            FollowerConstants.zeroPowerAccelerationMultiplier = 4.0
            FollowerConstants.centripetalScaling = 0.00008

            FollowerConstants.pathEndTimeoutConstraint = 300.0
            FollowerConstants.pathEndTValueConstraint = 0.995
            FollowerConstants.pathEndVelocityConstraint = 0.1
            FollowerConstants.pathEndTranslationalConstraint = 0.1
            FollowerConstants.pathEndHeadingConstraint = 0.007
        }
    }

    object RobotLocalizerConstants {
        init {
            ThreeWheelConstants.forwardTicksToInches = 0.00056050
            ThreeWheelConstants.strafeTicksToInches = -0.00056208
            ThreeWheelConstants.turnTicksToInches = 0.0005589587
            ThreeWheelConstants.leftY = 6.109
            ThreeWheelConstants.rightY = -6.109
            ThreeWheelConstants.strafeX = 6.391
            ThreeWheelConstants.leftEncoder_HardwareMapName = "fl"
            ThreeWheelConstants.rightEncoder_HardwareMapName = "fr"
            ThreeWheelConstants.strafeEncoder_HardwareMapName = "br"
            ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE
            ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD
            ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE
        }
    }

    // Drive Motors
    val frontLeft: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "fl")
    val backLeft: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "bl")
    val frontRight: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "fr")
    val backRight: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "br")

    // Other Motors
    val deliveryFront: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "delivery_front")
    val deliveryBack: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "delivery_back")

    // Servos
    val intakeSlide: Servo = hardwareMap.get(Servo::class.java, "intake_slide")
    val intakePivot: Servo = hardwareMap.get(Servo::class.java, "intake_pivot")
    val intakeWrist: Servo = hardwareMap.get(Servo::class.java, "intake_wrist")
    val intakeGripper: Servo = hardwareMap.get(Servo::class.java, "intake_gripper")
    val deliveryPivot: Servo = hardwareMap.get(Servo::class.java, "delivery_pivot")
    val deliveryGripper: Servo = hardwareMap.get(Servo::class.java, "delivery_gripper")
    val specimenGripper: Servo = hardwareMap.get(Servo::class.java, "specimen_gripper")

    // Limit Switches
    val deliveryLiftDownSwitch: AnalogInput = hardwareMap.get(AnalogInput::class.java, "delivery_lift_down")
    // Sensors
    val specimenDistanceSensor: DistanceSensor = hardwareMap.get(DistanceSensor::class.java,"specimen_sensor")

    // Servo Positions
    val deliveryGripperOpen = 0.2
    val deliveryGripperClosed = 0.0
    val deliveryPivotLow = 0.93
    val deliveryPivotMedium = 0.55
    val deliveryPivotHigh = 0.2

    val specimenDeliveryPosition = 1400
    val deliveryMaxHeight = 2500

    val specimenGripperOpen = 0.95
    val specimenGripperClosed = 0.4

    val intakeGripperNeutral = 0.26
    val intakeGripperClosedSides = 0.7
    val intakeGripperClosedTop = 0.0
    val intakeGripperClosedLoose = 0.66
    val intakeGripperClearance = .58

    val intakePivotDown = .675
    val intakePivotUp = 0.0
    val intakePivotMid = .25

    val intakeSlideMax = 1.0
    val intakeSlideMid = .55
    val intakeSlideMin = .37

    val intakeWristMid = 0.5
    val intakeWristLeft = 0.13
    val intakeWristRight = 0.85

    init {
        frontLeft.direction = DcMotorSimple.Direction.REVERSE
        backLeft.direction = DcMotorSimple.Direction.FORWARD
        frontRight.direction = DcMotorSimple.Direction.REVERSE
        backRight.direction = DcMotorSimple.Direction.REVERSE

        deliveryBack.direction = DcMotorSimple.Direction.REVERSE
        deliveryFront.direction = DcMotorSimple.Direction.REVERSE

        frontLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        frontRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        deliveryFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        deliveryBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        deliveryFront.mode = DcMotor.RunMode.RUN_USING_ENCODER
        deliveryBack.mode = DcMotor.RunMode.RUN_USING_ENCODER

        val pidNew = PIDFCoefficients(0.0007, 0.0, 0.0, 0.0)
        deliveryBack.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew)

        Constants.setConstants(RobotFollowerConstants::class.java, RobotLocalizerConstants::class.java)
    }

    val follower = Follower(hardwareMap)

    /**
     * X: Strafing left (-X) / right (+X)
     * Y: Moving forward (+Y) / backward (-Y)
     * Yaw: Clockwise (-Yaw) / counter-clockwise (+Yaw)
     */
    fun moveRobot(x: Double, y: Double, yaw: Double) {
        // Calculate wheel powers.

        var leftFrontPower = y + x - yaw
        var rightFrontPower = y - x + yaw
        var leftBackPower = y - x - yaw
        var rightBackPower = y + x + yaw

        // Normalize wheel powers to be less than 1.0
        var max = max(abs(leftFrontPower), abs(rightFrontPower))
        max = max(max, abs(leftBackPower))
        max = max(max, abs(rightBackPower))

        if (max > 1.0) {
            leftFrontPower /= max
            rightFrontPower /= max
            leftBackPower /= max
            rightBackPower /= max
        }

        // Send powers to the wheels.
        frontLeft.power = leftFrontPower
        frontRight.power = rightFrontPower
        backLeft.power = leftBackPower
        backRight.power = rightBackPower

    }

    fun moveLiftToPosition(targetPosition: Int, power: Double) {

        if(targetPosition > deliveryBack.currentPosition) {
            deliveryBack.power = -power
            deliveryFront.power = -power
        } else {
            deliveryBack.power = 0.0
            deliveryFront.power = 0.0
        }
    }

    fun  moveLiftToBottom() {
        if(deliveryBack.currentPosition > 300) {
            deliveryBack.power = 0.4
            deliveryFront.power = 0.4
        } else  {
            deliveryFront.power = 0.0
            deliveryBack.power = 0.0
        }
    }

    fun initializeInAuto() {
        intakeSlide.position = intakeSlideMin
        deliveryPivot.position = deliveryPivotHigh
        intakeWrist.position = intakeWristLeft
        deliveryGripper.position = deliveryGripperClosed
        intakeGripper.position = intakeGripperNeutral
        specimenGripper.position = specimenGripperClosed

    }

}