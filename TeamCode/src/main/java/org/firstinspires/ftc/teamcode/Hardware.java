package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

public class Hardware {
    public DcMotor led;

    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backLeftDrive;
    public DcMotor backRightDrive;
    public DcMotor leftTrackingWheel;
    public DcMotor rightTrackingWheel;
    public DcMotor middleTrackingWheel;
    public DcMotorEx arm;
    public Servo clawPivot;
    public Servo rightClaw;
    public Servo leftClaw;
    public Servo leftFoundation;
    public Servo rightFoundation;
    public Servo capstone;

    public static final int NEVEREST_TICKS_PER_REV = 1120;
    public static final int ARM_MAX = (int)(NEVEREST_TICKS_PER_REV * 0.65);
    public static final int ARM_MIN = 0;
    public static final int ARM_GRAB = 600;
    public static final int ARM_LIFT_1 = 500;
    public static final int ARM_LIFT_2 = 400;

    public static final double CLAW_CLOSED = 1;
    public static final double CLAW_OPEN = 0.3;

    public Hardware(HardwareMap hwMap) {
        led = hwMap.dcMotor.get("led");

        frontLeftDrive = hwMap.dcMotor.get("front_left");
        frontRightDrive = hwMap.dcMotor.get("front_right");
        backLeftDrive = hwMap.dcMotor.get("back_left");
        backRightDrive = hwMap.dcMotor.get("back_right");
        rightTrackingWheel = hwMap.dcMotor.get("right_tracking");
        middleTrackingWheel = backRightDrive;
        leftTrackingWheel = hwMap.dcMotor.get("left_tracking");
        leftTrackingWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = hwMap.get(DcMotorEx.class, "arm");
        clawPivot = hwMap.servo.get("pivot");
        clawPivot.setDirection(Servo.Direction.REVERSE);
        rightClaw = hwMap.servo.get("right");
        leftClaw = hwMap.servo.get("left");
        leftFoundation = hwMap.servo.get("left_fnd");
        rightFoundation = hwMap.servo.get("right_fnd");
        capstone = hwMap.servo.get("capstone");

        rightFoundation.setDirection(Servo.Direction.REVERSE);

        for (DcMotor motor : Arrays.asList(
                frontLeftDrive, frontRightDrive,
                backLeftDrive, backRightDrive, arm)) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Revert PIDF changes in SDK 5.3 to use SDK 5.2 coefficients
        // Fixes slow behavior on movement as seen from https://ftcforum.firstinspires.org/forum/control-hub-pilot-forum/77745-trouble-with-run_to_position-with-skystone-5-3-and-rev-firmware-1-8-2
        arm.setVelocityPIDFCoefficients(10, 3, 0, 0);
        arm.setPositionPIDFCoefficients(10);

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(0);

        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        rightClaw.setDirection(Servo.Direction.REVERSE);

        rightFoundation.setDirection(Servo.Direction.REVERSE);
    }
}
