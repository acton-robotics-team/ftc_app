package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

class Hardware(hwMap: HardwareMap) {

    var rightMotor: DcMotor
    var leftMotor: DcMotor
    var lifter: DcMotor

    init {
        rightMotor = hwMap.dcMotor.get("right_motor")
        leftMotor = hwMap.dcMotor.get("left_motor")
        lifter = hwMap.dcMotor.get("lifter")

        rightMotor.direction = DcMotorSimple.Direction.REVERSE
        lifter.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

    companion object {
        val SLOW_SPEED = 0.2
        val FULL_SPEED = 1.0

        val LIFTER_BOTTOM_POSITION = 0
        val LIFTER_TOP_POSITION = 3915
    }
}
