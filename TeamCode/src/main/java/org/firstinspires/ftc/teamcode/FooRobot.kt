package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

class FooRobot(val hardwareMap: HardwareMap){
    lateinit var frontLeft : DcMotor
    lateinit var frontRight : DcMotor
    lateinit var backLeft : DcMotor
    lateinit var backRight : DcMotor
    private lateinit var intake : DcMotor
    private lateinit var transfer: DcMotor
    lateinit var analogInput : AnalogInput
    val liftPressed get() = analogInput.let {
        it.voltage/it.maxVoltage
    } > 0.5


    fun setUp () {
        frontLeft = hardwareMap.dcMotor["fl"]
        frontRight = hardwareMap.dcMotor["fr"]
        backLeft = hardwareMap.dcMotor["bl"]
        backRight = hardwareMap.dcMotor["br"]
        intake = hardwareMap.dcMotor["intake"]
        transfer = hardwareMap.dcMotor["transfer"]
        analogInput = hardwareMap.analogInput["liftSwitch"]


        frontRight.direction = DcMotorSimple.Direction.REVERSE
        backRight.direction = (DcMotorSimple.Direction.REVERSE)

        intake.direction = DcMotorSimple.Direction.REVERSE
        transfer.direction = DcMotorSimple.Direction.REVERSE
    }

    fun driveForward(time: Long) {
        frontLeft.power = 1.0
        frontRight.power = 1.0
        backLeft.power = 1.0
        backRight.power = 1.0

        sleep(time)

    }
    fun sleep(milliseconds: Long) {
        try {
            Thread.sleep(milliseconds)
        } catch (e: InterruptedException) {
            Thread.currentThread().interrupt()
        }
    }

    fun startIntake(isEnabled:Boolean) {
        val power = if (isEnabled) {
            1.0
        } else {
            0.0
        }
        intake.power = power
        transfer.power = power
    }

}