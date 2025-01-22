package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs
import kotlin.math.max

class Robot(val hardwareMap: HardwareMap) {

    private val frontLeft: DcMotor = hardwareMap.get(DcMotor::class.java, "fl")
    private val backLeft: DcMotor = hardwareMap.get(DcMotor::class.java, "bl")
    private val frontRight: DcMotor = hardwareMap.get(DcMotor::class.java, "fr")
    private val backRight: DcMotor = hardwareMap.get(DcMotor::class.java, "br")

    init {
        frontLeft.direction = DcMotorSimple.Direction.FORWARD
        backLeft.direction = DcMotorSimple.Direction.FORWARD
        frontRight.direction = DcMotorSimple.Direction.REVERSE
        backRight.direction = DcMotorSimple.Direction.REVERSE
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