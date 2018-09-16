package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {
    public DcMotor rightMotor;
    public DcMotor leftMotor;

    public Hardware(HardwareMap hwMap) {
        rightMotor = hwMap.dcMotor.get("right_motor");
        leftMotor = hwMap.dcMotor.get("left_motor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
