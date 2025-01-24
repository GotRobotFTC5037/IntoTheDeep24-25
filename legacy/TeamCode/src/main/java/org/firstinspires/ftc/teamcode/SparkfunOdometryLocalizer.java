package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class SparkfunOdometryLocalizer implements Localizer {
    private final SparkFunOTOS odometrySensor;
    private Pose2d _poseEstimate = new Pose2d(0.0, 0.0, 0.0);
    private Pose2d _poseVelocity = new Pose2d();

    public SparkfunOdometryLocalizer(SparkFunOTOS odometrySensor) {
        this.odometrySensor = odometrySensor;
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return _poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d value) {
        // No operation
    }

    @Override
    public Pose2d getPoseVelocity() {
        return _poseVelocity;
    }

    @Override
    public void update() {
        _poseEstimate = new Pose2d(odometrySensor.getPosition().x, odometrySensor.getPosition().y, odometrySensor.getPosition().h);
        _poseVelocity = new Pose2d(odometrySensor.getVelocity().x, odometrySensor.getVelocity().y, odometrySensor.getVelocity().h);
    }
}

