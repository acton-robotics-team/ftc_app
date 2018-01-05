package com.github.actonroboticsteam.ftcapp

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime

import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer

import java.util.concurrent.FutureTask
import java.util.concurrent.TimeUnit
import java.util.concurrent.TimeoutException
import kotlin.math.roundToInt

@Autonomous(name = "EZAutonomous program")
class EzAutonomousMode : LinearOpMode() {
    private val runtime = ElapsedTime()
    @Volatile private var logs = ""
    @Synchronized private fun addLogLine(text: String) {
        logs += "[${runtime.time(TimeUnit.SECONDS)}] $text\n"
        telemetry.addLine(logs)
        telemetry.update()
    }

    private fun sleep() = if (!opModeIsActive()) {
        throw OpModeStoppedException()
    } else {
        idle()
    }

    private fun drive(robot: RobotConfig, rotations: Double) {
        val encoderTicks = (rotations * RobotConfig.TETRIX_TICKS_PER_REVOLUTION).roundToInt()

        addLogLine("Driving for $rotations rotations, which is $encoderTicks ticks")

        val oldMode = robot.leftDriveMotor.mode

        robot.leftDriveMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        robot.leftDriveMotor.mode = DcMotor.RunMode.RUN_TO_POSITION

        robot.leftDriveMotor.targetPosition = encoderTicks

        robot.leftDriveMotor.power = if (encoderTicks > 0) 0.2 else -0.2
        robot.rightDriveMotor.power = if (encoderTicks > 0) 0.2 else -0.2
        while (robot.leftDriveMotor.isBusy) {
            sleep()
        }
        robot.leftDriveMotor.power = 0.0
        robot.rightDriveMotor.power = 0.0
        robot.leftDriveMotor.mode = oldMode
    }

    override fun runOpMode() {
        val robot = RobotConfig(hardwareMap)

        robot.leftTopGrabberServo.position = RobotConfig.GRABBER_GRABBED
        robot.rightTopGrabberServo.position = RobotConfig.GRABBER_GRABBED
        robot.leftBottomGrabberServo.position = RobotConfig.GRABBER_GRABBED
        robot.rightBottomGrabberServo.position = RobotConfig.GRABBER_GRABBED

        // wait for the start button to be pressed.
        waitForStart()

        runtime.reset()

        // 1. Hit jewel & scan pictogram simultaneously
        // 2. Move backward and detect cryptobox columns with ODS
        // 3. Turn and move forward to put glyph in cryptobox
        // 4. Release glyph

        try {
            drive(robot, 2.8) // to safe zone
        } catch (e: Exception) {
            addLogLine("HIT EXCEPTION. Stopping op mode.")
            addLogLine("Exception backtrace:")
            addLogLine(e.stackTrace.contentToString())
        }
    }
}