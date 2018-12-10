package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor

/**
 * Move to position specified by encoderTicks and block until finished.
 */
fun DcMotor.moveToPosition(encoderTicks: Int, power: Double, maintainPosition: Boolean) {
    val oldMode = this.mode
    this.mode = DcMotor.RunMode.RUN_TO_POSITION
    this.power = power
    this.targetPosition = encoderTicks

    while (this.isBusy) {
        Thread.yield()
    }

    if (!maintainPosition) {
        this.power = 0.0
        this.mode = oldMode
    }
}