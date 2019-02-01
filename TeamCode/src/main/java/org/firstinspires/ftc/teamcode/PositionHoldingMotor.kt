package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor


class PositionHoldingMotor(private val motor: DcMotor) {
    fun processInput(input: Float) {
        motor.apply {
            mode = DcMotor.RunMode.RUN_TO_POSITION
            power = 0.5
            if (input > 0.1) {
                targetPosition = currentPosition + 50
            } else if (input < 0.1) {
                targetPosition = currentPosition - 50
            }
        }
    }

    fun setTargetPosition(targetPosition: Int) {
        motor.targetPosition = targetPosition
    }
}