package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="OTOSTestAuto", group="auto")
//@Disabled
public class OTOSTestAuto extends LinearOpMode {

    Hardware robot = new Hardware();

    private static final double MAX_SPEED = 0.2;  // Maximum speed of the robot
    private static final double TOLERANCE = 0.1;  // Acceptable distance from target to stop
    private static final double ANGLE_TOLERANCE = 2.0;
    private static final double X_THRESHOLD = 1;
    private static final double Y_THRESHOLD = 1;
    private static final double HEADING_THRESHOLD = 3;
    private static final double INCHES_SLOW_DOWN = 10;
    private static final double SLOW_DOWN_MULTIPLIER = 0.15;

    SparkFunOTOS.Pose2D pos;

    @Override
    public void runOpMode() {


//        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        robot.init(hardwareMap);
        Hardware.initializeDriveMotors(robot);
        configureOtos();
        sleep(1000);


        while(!isStarted()) {
            // Wait for the game to start (driver presses PLAY)
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        moveToTarget(-1, 0, 0);
        sleep(500);

        sleep(1000);
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // stored in the sensor, it's part of the library, so you need to set at the
        // start of all your programs.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        robot.odometrySensor.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        robot.odometrySensor.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-3.75, -7.5, 90); // should be -3.75 & -7.5 and 90
        robot.odometrySensor.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        robot.odometrySensor.setLinearScalar(1);
        robot.odometrySensor.setAngularScalar(0.992);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your programs. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        robot.odometrySensor.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        robot.odometrySensor.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        robot.odometrySensor.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        robot.odometrySensor.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured");
        telemetry.update();
    }
    public void moveToTarget(double targetX, double targetY, double targetHeading) {
        SparkFunOTOS.Pose2D currentPos = myPosition();

        double currentX = currentPos.x;  // Retrieve current x position from sensor
        double currentY = currentPos.y;  // Retrieve current y position from sensor
        double currentHeading = currentPos.h;  // Retrieve current heading from sensor

        // Error calculations
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        double deltaHeading = normalizeAngle(targetHeading - currentHeading);  // Angle difference

        // Control loop to move robot
        while (distanceToTarget > TOLERANCE || Math.abs(deltaHeading) > ANGLE_TOLERANCE) {
             currentX = currentPos.x;  // Retrieve current x position from sensor
             currentY = currentPos.y;  // Retrieve current y position from sensor
             currentHeading = currentPos.h;

            deltaX = targetX - currentX;
            deltaY = targetY - currentY;
            distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            deltaHeading = normalizeAngle(targetHeading - currentHeading);

            // Move robot towards target
            double speed = Math.min(MAX_SPEED, distanceToTarget);  // Move speed based on distance
            double strafe = Math.atan2(deltaY, deltaX);  // Direction to target
            double forward = speed * Math.cos(strafe);  // Forward component of movement
            double sideways = speed * Math.sin(strafe);  // Strafing component of movement

            // Adjust robot's heading to target heading
            double turnSpeed = 0.2 * deltaHeading;  // Simple proportional control for heading

            // Calculate motor speeds for each of the 4 wheels based on movement
            double flSpeed = forward + sideways + turnSpeed;  // Front left motor speed
            double frSpeed = forward - sideways - turnSpeed;  // Front right motor speed
            double blSpeed = forward - sideways + turnSpeed;  // Back left motor speed
            double brSpeed = forward + sideways - turnSpeed;  // Back right motor speed

            // Normalize speeds to ensure they stay within the max speed limit
            double maxMotorSpeed = Math.max(Math.abs(flSpeed), Math.max(Math.abs(frSpeed), Math.max(Math.abs(blSpeed), Math.abs(brSpeed))));
            if (maxMotorSpeed > 1.0) {
                flSpeed /= maxMotorSpeed;
                frSpeed /= maxMotorSpeed;
                blSpeed /= maxMotorSpeed;
                brSpeed /= maxMotorSpeed;
            }

            // Send calculated speeds to motors
            robot.frontLeft.setPower(flSpeed);
            robot.frontRight.setPower(frSpeed);
            robot.backLeft.setPower(blSpeed);
            robot.backRight.setPower(brSpeed);

            // Small delay to allow motors to update
            try {
                Thread.sleep(20);  // 20ms delay for motor commands to take effect
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            telemetry.addData("Current X", currentX);
            telemetry.addData("Current Y", currentY);
            telemetry.addData("X Delta", deltaX);
            telemetry.addData("Y Delta", deltaX);
            telemetry.addData("Target X", targetX);
            telemetry.addData("Target Y", targetY);
            telemetry.update();
        }

        // Stop the robot when target is reached
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    SparkFunOTOS.Pose2D myPosition() {
        pos = robot.odometrySensor.getPosition();
        SparkFunOTOS.Pose2D myPos = new SparkFunOTOS.Pose2D(pos.x, pos.y, -pos.h);
        return(myPos);
    }
}
