package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware {

    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    public DcMotor deliveryLiftMain = null;
    public DcMotor deliveryLiftAux = null;
    public DcMotor intakeArm = null;
    public DcMotor lift = null;
    public Servo stars = null;
    public Servo specimenGripper = null;
    public Servo escapement = null;
    public Servo kickstand = null;
    public Servo bucket = null;
    public Servo hookRelease = null;
    public AnalogInput intakeLimitSwitch = null;
    public AnalogInput mainLiftDownLimitSwitch = null;
    public AnalogInput mainLiftUpLimitSwitch = null;
    public AnalogInput auxLiftDownLimitSwitch = null;
    public AnalogInput auxLiftUpLimitSwitch = null;

    public SparkFunOTOS odometrySensor;

    public OTOSMecanumDrive drive;

    static HardwareMap hardwareMap = null;

    public Hardware() {

    }

    public double escapementOpen = 0.5;
    public double escapementClosed = 0;
    public double kickstandDown = 0;
    public double specimenGripperUngrip = 0;
    public double sensorMultipler = 1.72;
    private ElapsedTime runtime = new ElapsedTime();


    public void init(HardwareMap hardwareMap) {

        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        deliveryLiftMain = hardwareMap.get(DcMotor.class, "delivery_big");
        deliveryLiftAux = hardwareMap.get(DcMotor.class, "delivery_small");
        intakeArm = hardwareMap.get(DcMotor.class, "intake");
        lift = hardwareMap.get(DcMotor.class, "lift");

        stars = hardwareMap.get(Servo.class, "stars");
        specimenGripper = hardwareMap.get(Servo.class, "gripper");
        escapement = hardwareMap.get(Servo.class, "escapement");
        kickstand = hardwareMap.get(Servo.class, "kickstand");
        bucket = hardwareMap.get(Servo.class, "bucket");
        hookRelease = hardwareMap.get(Servo.class, "hook_release");
//        wrist = hardwareMap.get(Servo.class, "wrist");

        odometrySensor = hardwareMap.get(SparkFunOTOS.class, "OTOS");

//        drive = new OTOSMecanumDrive(frontLeft, frontRight, backLeft, backRight, odometrySensor, batteryVoltageSensor);

        mainLiftDownLimitSwitch = hardwareMap.get(AnalogInput.class, "main_lift_down");
        mainLiftUpLimitSwitch = hardwareMap.get(AnalogInput.class, "main_lift_up");
        auxLiftDownLimitSwitch = hardwareMap.get(AnalogInput.class, "aux_lift_down");
        auxLiftUpLimitSwitch = hardwareMap.get(AnalogInput.class, "aux_lift_up");
        intakeLimitSwitch = hardwareMap.get(AnalogInput.class, "intake_switch");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        deliveryLiftMain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        deliveryLiftAux.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    static void initializeDriveMotors(Hardware robot) {
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.stars.setDirection(Servo.Direction.FORWARD);

        robot.odometrySensor.calibrateImu();
        robot.odometrySensor.setAngularScalar(0.997707296347086);
        robot.odometrySensor.setLinearScalar(1);
        robot.odometrySensor.setOffset(new SparkFunOTOS.Pose2D(2.25,7.75,0));
        robot.odometrySensor.resetTracking();

    }
}

