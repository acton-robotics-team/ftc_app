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
    public DcMotor leftIntake;
    public DcMotor rightIntake;

    public Hardware(HardwareMap hwMap) {
        led = hwMap.dcMotor.get("led");

        frontLeftDrive = hwMap.dcMotor.get("front_left");
        frontRightDrive = hwMap.dcMotor.get("front_right");
        backLeftDrive = hwMap.dcMotor.get("back_left");
        backRightDrive = hwMap.dcMotor.get("back_right");
        leftIntake = hwMap.dcMotor.get("left_intake");
        rightIntake = hwMap.dcMotor.get("right_intake");

        for (DcMotor motor : Arrays.asList(
                frontLeftDrive, frontRightDrive,
                backLeftDrive, backRightDrive)) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        for (DcMotor motor : Arrays.asList(frontLeftDrive, frontRightDrive, backRightDrive, backLeftDrive)) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
