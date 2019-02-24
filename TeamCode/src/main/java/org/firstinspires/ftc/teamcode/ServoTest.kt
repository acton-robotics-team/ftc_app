package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "Servo test program")
class ServoTest : LinearOpMode() {
    var position = 0.0

    override fun runOpMode() {
        val hw = Hardware(hardwareMap, this)
        waitForStart()

        while (opModeIsActive()) {

            hw.rightArmSupporter.position = position
        }
    }
}