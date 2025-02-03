package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ftc.LazyImu
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.VoltageSensor
import kotlin.math.abs
import kotlin.math.max


class Robot(val hardwareMap: HardwareMap) {

    // Drive Motors
    private val frontLeft: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "fl")
    private val backLeft: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "bl")
    private val frontRight: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "fr")
    private val backRight: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "br")

    // Other Motors
    public val deliveryFront: DcMotor = hardwareMap.get(DcMotor::class.java, "delivery_front")
    public val deliveryBack: DcMotor = hardwareMap.get(DcMotor::class.java, "delivery_back")
//    private val winch1: DcMotor = hardwareMap.get(DcMotor::class.java, "winch1")
//    private val winch2: DcMotor = hardwareMap.get(DcMotor::class.java, "winch2")

    // Servos
    public val intakeSlide: Servo = hardwareMap.get(Servo::class.java, "intake_slide")
    public val intakePivot: Servo = hardwareMap.get(Servo::class.java, "intake_pivot")
    public val intakeWrist: Servo = hardwareMap.get(Servo::class.java, "intake_wrist")
    public val intakeGripper: Servo = hardwareMap.get(Servo::class.java, "intake_gripper")
    public val deliveryPivot: Servo = hardwareMap.get(Servo::class.java, "delivery_pivot")
    public val deliveryGripper: Servo = hardwareMap.get(Servo::class.java, "delivery_gripper")
    public val specimenGripper: Servo = hardwareMap.get(Servo::class.java, "specimen_gripper")
//    public val winchServo: Servo = hardwareMap.get(Servo::class.java, "winch_servo")

    // Limit Switches
    public val deliveryLiftDownSwitch: AnalogInput = hardwareMap.get(AnalogInput::class.java, "delivery_lift_down")

    // Sensors
    public val specimenDistanceSensor: DistanceSensor = hardwareMap.get(DistanceSensor::class.java,"specimen_sensor")
    public val transferDistanceSensor: ColorRangeSensor = hardwareMap.get(ColorRangeSensor::class.java, "transfer_sensor")

    // Servo Positions
    public val deliveryGripperOpen = 0.2
    public val deliveryGripperClosed = 0.0
    public val deliveryPivotLow = 0.27
    public val deliveryPivotMedium = 0.62
    public val deliveryPivotHigh = 0.95

    public val specimenGripperOpen = 0.95
    public val specimenGripperClosed = 0.3

    public val intakeGripperNeutral = 0.26
    public val intakeGripperClosedSides = 0.7
    public val intakeGripperClosedTop = 0.0
    public val intakeGripperClosedLoose = 0.66

    public val intakePivotDown = .675
    public val intakePivotUp = 0.0

    public val intakeSlideMax = 0.0
    public val intakeSlideMin = .54

    public val intakeWristMid = 0.475
    public val intakeWristLeft = 0.8
    public val intakeWristRight = 0.15




    // Roadrunner
//    val roadrunnerMecanumDrive: MecanumDrive
//    private val voltageSensor: VoltageSensor = hardwareMap.get(VoltageSensor::class.java,"Control Hub")
//    private val imu: IMU = hardwareMap.get(IMU::class.java,"imu")
//    private val par0: OverflowEncoder = hardwareMap.get(OverflowEncoder::class.java, "par0")
//    private val par1: OverflowEncoder = hardwareMap.get(OverflowEncoder::class.java, "par1")
//    private val perp: OverflowEncoder = hardwareMap.get(OverflowEncoder::class.java, "perp")
//    private val pose: Pose2d = hardwareMap.get(Pose2d::class.java, "pose")

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

//        roadrunnerMecanumDrive = MecanumDrive(frontLeft,backLeft,backRight,frontRight,imu,voltageSensor,par1,par0,perp,pose)

//        stars.direction = Servo.Direction.FORWARD
    }

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

}