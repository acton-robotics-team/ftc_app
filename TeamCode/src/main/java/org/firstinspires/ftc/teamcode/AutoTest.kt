package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous(name = "Autonomous TEST program")
class AutoTest : LinearOpMode() {
    override fun runOpMode() {
        val hw = Hardware(hardwareMap, this)
        waitForStart()

        hw.rotateArmFromStartPosition(50f)

        while (opModeIsActive()) {
        }
    }
}