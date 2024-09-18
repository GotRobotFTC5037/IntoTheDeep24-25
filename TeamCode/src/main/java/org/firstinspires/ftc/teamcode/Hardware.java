package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {

    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    HardwareMap hardwareMap = null;

    public void init(HardwareMap hardwareMap) {

        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    static void initializeDriveMotors(Hardware robotObj) {
        robotObj.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotObj.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotObj.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotObj.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robotObj.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotObj.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotObj.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotObj.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

