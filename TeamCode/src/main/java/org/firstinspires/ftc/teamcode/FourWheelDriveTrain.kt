package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import edu.spa.ftclib.internal.controller.FinishableIntegratedController
import edu.spa.ftclib.internal.drivetrain.HeadingableTankDrivetrain

class FourWheelDriveTrain(val backLeft: DcMotor, val backRight: DcMotor, val frontLeft: DcMotor, val frontRight: DcMotor, val controller: FinishableIntegratedController) : HeadingableTankDrivetrain(backLeft, backRight, controller) {
    override fun calculateMotorPowers(): DoubleArray {
        val powers = super.calculateMotorPowers()
        powers[0] *= 0.5
        powers[1] *= 0.5
        return powers
    }
    override fun updateMotorPowers() {
        super.updateMotorPowers()
        val motorPowers = calculateMotorPowers()
        frontLeft.power = motorPowers[0]
        frontRight.power = motorPowers[1]
    }
}