package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Park Only Net Zone Auto", group = "Auto")
public class ParkOnlyNetZone extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode() {

        // Auto Initialization Starts:

        robot.init(hardwareMap);

        Hardware.initializeDriveMotors(robot);

        robot.kickstand.setPosition(robot.kickstandDown);
        robot.deliveryLiftMain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        resetRuntime();
        if (isStopRequested()) return;
        sleep(500);

        // Code starts here
        // ALL CODE ASSUMES SPECIMEN SIDE OF ROBOT IS FRONT (according to starting position in auto)

        sleep(15000);
        diagonalStrafe(0, -0.5, 4);
        sleep(100);
    }
    public void diagonalStrafe(double forwardPower, double strafePower, int timeInSeconds) {
        // Assuming positive forwardPower moves the robot forward
        // Assuming positive strafePower moves the robot to the right
        // The combination of both movements results in a diagonal path.

        // Motor assignments for diagonal movement (forward and right)
        robot.frontLeft.setPower(-forwardPower + strafePower);  // Front-left motor (forward + right)
        robot.frontRight.setPower(-forwardPower - strafePower);  // Front-right motor (forward - right)
        robot.backLeft.setPower(-forwardPower - strafePower);  // Back-left motor (forward - right)
        robot.backRight.setPower(-forwardPower + strafePower);  // Back-right motor (forward + right)

        // Wait for the given period (convert seconds to milliseconds)
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < timeInSeconds * 1000) {
            // Keep the motors running for the specified time
        }

        // Stop all motors after the time has passed
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
    }

}
