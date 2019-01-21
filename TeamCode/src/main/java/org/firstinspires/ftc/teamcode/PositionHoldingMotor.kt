package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor


class PositionHoldingMotor(motor: DcMotor, power: Double) {
    private val motor = motor
    private val power = power
    private var lastTargetPosition = 0
    private var previouslyOnManualControl = false

    fun processInput(input: Float) {
        if (Math.abs(input) > 0.1) {
            previouslyOnManualControl = true
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            motor.power = power * input
        } else {
            motor.mode = DcMotor.RunMode.RUN_TO_POSITION
            motor.power = power

            if (previouslyOnManualControl) {
                previouslyOnManualControl = false
                lastTargetPosition = motor.currentPosition
            }

            motor.targetPosition = lastTargetPosition
        }
    }

    fun setTargetPosition(targetPosition: Int) {
        lastTargetPosition = targetPosition
    }
}