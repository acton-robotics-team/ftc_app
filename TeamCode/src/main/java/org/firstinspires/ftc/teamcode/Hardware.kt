package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

class Hardware(hwMap: HardwareMap) {

    val rightDrive: DcMotor = hwMap.dcMotor.get("right_motor")
    val leftDrive: DcMotor = hwMap.dcMotor.get("left_motor")
    val lifter: DcMotor = hwMap.dcMotor.get("lifter")
//    val arm: DcMotor = hwMap.dcMotor.get("arm")
    val armExtender: DcMotor = hwMap.dcMotor.get("arm_extender")

    init {
        rightDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightDrive.direction = DcMotorSimple.Direction.REVERSE

        // Zero encoders
        lifter.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        lifter.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    companion object {
        const val DRIVE_SLOW = 0.5
        const val DRIVE_FAST = 1.0

        const val LIFTER_BOTTOM_POSITION = 0
        const val LIFTER_TOP_POSITION = 5200

        const val ARM_DOWN_POSITION = 0 // placeholder
    }
}
