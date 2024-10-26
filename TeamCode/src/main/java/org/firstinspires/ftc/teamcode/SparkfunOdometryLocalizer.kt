package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.qualcomm.hardware.sparkfun.SparkFunOTOS

class SparkfunOdometryLocalizer (
        private val odometrySensor: SparkFunOTOS
): Localizer {
    private var _poseEstimate = Pose2d(x=0.0, y=0.0, heading=0.0)
    private var _poseVelocity = Pose2d()
    override var poseEstimate: Pose2d
        get() = _poseEstimate
        set(value) {}
    override val poseVelocity: Pose2d?
        get() = _poseVelocity

    override fun update() {
        _poseEstimate = Pose2d(x=odometrySensor.position.x, y=odometrySensor.position.y, heading=odometrySensor.position.h)
        _poseVelocity = Pose2d(x=odometrySensor.velocity.x, y=odometrySensor.velocity.y, heading=odometrySensor.velocity.h)
    }
}