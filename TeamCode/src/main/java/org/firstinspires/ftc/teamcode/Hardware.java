package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    public DcMotor leftArm;
    public DcMotor rightArm;
    public DcMotor armExtender;
    public Servo grabberPivot;
    public Servo grabber;
    public Servo armHolder;

    public static final double GRABBER_PIVOT_POS_GRAB = 1.0;
    public static final double GRABBER_PIVOT_POS_RELEASE = 0.7;

    public static final double GRABBER_OPEN = 1.0;
    public static final double GRABBER_CLOSED = 0.0;

    public static final double ARM_HOLDER_HOLDING = 1.0;
    public static final double ARM_HOLDER_RETRACTED = 0.5;

    public Hardware(HardwareMap hwMap) {
        led = hwMap.dcMotor.get("led");

        frontLeftDrive = hwMap.dcMotor.get("front_left");
        frontRightDrive = hwMap.dcMotor.get("front_right");
        backLeftDrive = hwMap.dcMotor.get("back_left");
        backRightDrive = hwMap.dcMotor.get("back_right");
        leftArm = hwMap.dcMotor.get("left_arm");
        rightArm = hwMap.dcMotor.get("right_arm");
        armExtender = hwMap.dcMotor.get("extender");

        grabberPivot = hwMap.servo.get("pivot");
        grabber = hwMap.servo.get("grabber");
        armHolder = hwMap.servo.get("holder");

        for (DcMotor motor : Arrays.asList(
                frontLeftDrive, frontRightDrive,
                backLeftDrive, backRightDrive,
                leftArm, rightArm, armExtender)) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        for (DcMotor motor : Arrays.asList(frontLeftDrive, frontRightDrive, backRightDrive, backLeftDrive)) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftArm.setDirection(DcMotorSimple.Direction.REVERSE);

        leftArm.setTargetPosition(0);
        rightArm.setTargetPosition(0);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setArmPosition(int encoderTicks) {
        for (DcMotor motor : Arrays.asList(leftArm, rightArm)) {
            motor.setPower(0.7);
            motor.setTargetPosition(encoderTicks);
        }
    }

    private static final double INITIAL_ANGLE = 0.25 * Math.PI; // TODO analyze

    /**
     * Sets arm angle from angle (rad) above the horizontal
     * @param angleRad
     */
    public void setArmAngle(double angleRad) {
        double rad = angleRad - INITIAL_ANGLE;
        int encoderTicks = (int) (rad / (2 * Math.PI) * leftArm.getMotorType().getTicksPerRev());
        setArmPosition(encoderTicks);
    }
}
