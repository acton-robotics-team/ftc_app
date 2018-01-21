package com.github.actonroboticsteam.ftcapp

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor

@Autonomous(name = "Test program")
class TestMode : LinearOpMode() {
    override fun runOpMode() {
        val robot = RobotConfig(hardwareMap)

        // wait for the start button to be pressed.
        waitForStart()

        while (opModeIsActive()) {
            robot.leftDriveMotor.power = 1.0
            robot.rightDriveMotor.power = 1.0
            idle()
        }
    }
}