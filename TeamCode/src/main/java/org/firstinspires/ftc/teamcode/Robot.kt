package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.abs
import kotlin.math.max


class Robot(val hardwareMap: HardwareMap) {

    // Drive Motors
    public val frontLeft: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "fl")
    public val backLeft: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "bl")
    public val frontRight: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "fr")
    public val backRight: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "br")

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
    public val deliveryPivotLow = 0.93
    public val deliveryPivotMedium = 0.55
    public val deliveryPivotHigh = 0.2

    public val specimenDeliveryPosition = 1300
    public val deliveryMaxHeight = 2500

    public val specimenGripperOpen = 0.95
    public val specimenGripperClosed = 0.4

    public val intakeGripperNeutral = 0.26
    public val intakeGripperClosedSides = 0.7
    public val intakeGripperClosedTop = 0.0
    public val intakeGripperClosedLoose = 0.66
    public val intakeGripperClearance = .58

    public val intakePivotDown = .675
    public val intakePivotUp = 0.0
    public val intakePivotMid = .25

    public val intakeSlideMax = 1.0
    public val intakeSlideMid = .55
    public val intakeSlideMin = .37

    public val intakeWristMid = 0.5
    public val intakeWristLeft = 0.13
    public val intakeWristRight = 0.85

    var inPerTick: Double = 0.00110697703 * 903.361111296


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

//        frontLeft.mode = DcMotor.RunMode.RUN_USING_ENCODER
//        backLeft.mode = DcMotor.RunMode.RUN_USING_ENCODER
//        frontRight.mode = DcMotor.RunMode.RUN_USING_ENCODER
//        backRight.mode = DcMotor.RunMode.RUN_USING_ENCODER

        deliveryFront.mode = DcMotor.RunMode.RUN_USING_ENCODER
        deliveryBack.mode = DcMotor.RunMode.RUN_USING_ENCODER

//        deliveryBack.mode = DcMotor.RunMode.RUN_TO_POSITION
//        roadrunnerMecanumDrive = MecanumDrive(frontLeft,backLeft,backRight,frontRight,lazyImu,voltageSensor,par1,par0,perp,pose)

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

    fun moveDiagonally(
        power: Double,
        timeMs: Long,
        direction: String
    ) {
        val clippedPower = power.coerceIn(-1.0, 1.0)

        // Set motor powers for diagonal movement
        when (direction.lowercase()) {
            "forward_left" -> {
                frontLeft.power = 0.0
                frontRight.power = clippedPower
                backLeft.power = clippedPower
                backRight.power = 0.0
            }
            "forward_right" -> {
                frontLeft.power = clippedPower
                frontRight.power = 0.0
                backLeft.power = 0.0
                backRight.power = clippedPower
            }
            "backward_left" -> {
                frontLeft.power = -clippedPower
                frontRight.power = 0.0
                backLeft.power = 0.0
                backRight.power = -clippedPower
            }
            "backward_right" -> {
                frontLeft.power = 0.0
                frontRight.power = -clippedPower
                backLeft.power = -clippedPower
                backRight.power = 0.0
            }
            else -> {
                println("Invalid direction. Use 'forward_left', 'forward_right', 'backward_left', or 'backward_right'.")
                return
            }
        }

        val startTime = System.currentTimeMillis()
        while (System.currentTimeMillis() - startTime < timeMs) {
            frontLeft.power += 0
            frontRight.power -= 0
            backLeft.power += 0
            backRight.power -= 0
        }

        // Stop motors after the time has elapsed
        frontLeft.power = 0.0
        frontRight.power = 0.0
        backLeft.power = 0.0
        backRight.power = 0.0
    }

    fun turnRobot(power: Double, timeMs: Long) {
        // Set opposite power to turn
        frontLeft.power = power
        backLeft.power = power
        frontRight.power = -power
        backRight.power = -power // Opposite direction for turning

       Thread.sleep(timeMs) // Wait for the turn to complete

        // Stop motors after turning
        frontLeft.power = 0.0
        backLeft.power = 0.0
        frontRight.power = 0.0
        backRight.power = 0.0
    }

    fun moveLiftToPosition(targetPosition: Int, power: Double) {

        deliveryBack.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        while (deliveryBack.currentPosition <= targetPosition) {
            deliveryBack.power = -power
            deliveryFront.power = -power
        }
        deliveryBack.power = 0.0
        deliveryFront.power = 0.0

    }

    fun moveForward(x: Double, y: Double, yaw: Double, inches: Double) {
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

        if (frontLeft.currentPosition > inches){
            frontLeft.power = 0.0
            frontRight.power = 0.0
            backLeft.power = 0.0
            backRight.power = 0.0

        }
        inches * inPerTick

        // Send powers to the wheels.
        frontLeft.power = leftFrontPower
        frontRight.power = rightFrontPower
        backLeft.power = leftBackPower
        backRight.power = rightBackPower
    }

    fun initializeInAuto() {
        intakeSlide.position = intakeSlideMin
        deliveryPivot.position = deliveryPivotLow
        intakeWrist.position = intakeWristLeft
        deliveryGripper.position = deliveryGripperClosed
        intakeGripper.position = intakeGripperNeutral
    }
}