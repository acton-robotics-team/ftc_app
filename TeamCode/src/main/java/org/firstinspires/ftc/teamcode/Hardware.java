package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;

public class Hardware {
    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backLeftDrive;
    public DcMotor backRightDrive;
    public DcMotor leftArm;
    public DcMotor rightArm;
    private DcMotor led;

    public Hardware(HardwareMap hwMap) {
        frontLeftDrive = hwMap.dcMotor.get("front_left");
        frontRightDrive = hwMap.dcMotor.get("front_right");
        backLeftDrive = hwMap.dcMotor.get("back_left");
        backRightDrive = hwMap.dcMotor.get("back_right");
        leftArm = hwMap.dcMotor.get("left_arm");
        rightArm = hwMap.dcMotor.get("right_arm");
        led = hwMap.dcMotor.get("led");

        for (DcMotor motor : Arrays.asList(
                frontLeftDrive, frontRightDrive,
                backLeftDrive, backRightDrive,
                leftArm, rightArm)) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftArm.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setLed(double brightness) {
        led.setPower(brightness);
    }
}
