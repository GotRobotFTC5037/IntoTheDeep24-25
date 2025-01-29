package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.abs
import kotlin.math.max

class Robot(val hardwareMap: HardwareMap) {

    // Drive Motors
    private val frontLeft: DcMotor = hardwareMap.get(DcMotor::class.java, "fl")
    private val backLeft: DcMotor = hardwareMap.get(DcMotor::class.java, "bl")
    private val frontRight: DcMotor = hardwareMap.get(DcMotor::class.java, "fr")
    private val backRight: DcMotor = hardwareMap.get(DcMotor::class.java, "br")

    // Other Motors
    private val delivery1: DcMotor = hardwareMap.get(DcMotor::class.java, "delivery1")
    private val delivery2: DcMotor = hardwareMap.get(DcMotor::class.java, "delivery2")
    private val winch1: DcMotor = hardwareMap.get(DcMotor::class.java, "winch1")
    private val winch2: DcMotor = hardwareMap.get(DcMotor::class.java, "winch2")

    // Servos
    private val intakeSlide: Servo = hardwareMap.get(Servo::class.java, "intake_slide")
    private val intakePivot: Servo = hardwareMap.get(Servo::class.java, "intake_pivot")
    private val intakeWrist: Servo = hardwareMap.get(Servo::class.java, "intake_wrist")
    private val intakeFingers: Servo = hardwareMap.get(Servo::class.java, "intake_fingers")
    private val deliveryPivot: Servo = hardwareMap.get(Servo::class.java, "delivery_pivot")
    private val deliveryFingers: Servo = hardwareMap.get(Servo::class.java, "delivery_fingers")
    private val specimenFingers: Servo = hardwareMap.get(Servo::class.java, "specimen_fingers")
    private val winchServo: Servo = hardwareMap.get(Servo::class.java, "winch_servo")

    // Limit Switches
    private val intakeSwitch: AnalogInput = hardwareMap.get(AnalogInput::class.java, "intake_switch")
    private val mainLiftDownSwitch: AnalogInput = hardwareMap.get(AnalogInput::class.java, "main_lift_down")
    private val mainLiftUpSwitch: AnalogInput = hardwareMap.get(AnalogInput::class.java, "main_lift_up")
    private val auxLiftUpSwitch: AnalogInput = hardwareMap.get(AnalogInput::class.java, "aux_lift_up")
    private val auxLiftDownSwitch: AnalogInput = hardwareMap.get(AnalogInput::class.java, "aux_lift_down")


    init {
        frontLeft.direction = DcMotorSimple.Direction.FORWARD
        backLeft.direction = DcMotorSimple.Direction.FORWARD
        frontRight.direction = DcMotorSimple.Direction.REVERSE
        backRight.direction = DcMotorSimple.Direction.REVERSE

        frontLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        frontRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        delivery1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        delivery2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

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