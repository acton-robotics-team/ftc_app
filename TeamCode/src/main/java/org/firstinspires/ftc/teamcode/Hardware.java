package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {
    public static final double SLOW_SPEED = 0.2;
    public static final double FULL_SPEED = 1.0;

    public static final int LIFTER_BOTTOM_POSITION = 0;
    public static final int LIFTER_TOP_POSITION = 3915;

    public DcMotor rightMotor;
    public DcMotor leftMotor;
    public DcMotor lifter;

    public Hardware(HardwareMap hwMap) {
        rightMotor = hwMap.dcMotor.get("right_motor");
        leftMotor = hwMap.dcMotor.get("left_motor");
        lifter = hwMap.dcMotor.get("lifter");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
