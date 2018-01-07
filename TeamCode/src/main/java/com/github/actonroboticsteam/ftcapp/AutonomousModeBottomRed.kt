package com.github.actonroboticsteam.ftcapp

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime

import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.pattonvillerobotics.commoncode.robotclasses.opencv.JewelColorDetector
import org.pattonvillerobotics.commoncode.robotclasses.opencv.util.PhoneOrientation
import org.pattonvillerobotics.commoncode.robotclasses.vuforia.VuforiaNavigation
import org.pattonvillerobotics.commoncode.robotclasses.vuforia.VuforiaParameters

import java.util.concurrent.FutureTask
import java.util.concurrent.TimeUnit
import java.util.concurrent.TimeoutException
import kotlin.math.roundToInt

@Autonomous(name = "EZAutonomous program: bottom red")
class AutonomousModeBottomRed : LinearOpMode() {
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
        // encoder ticks and direction reversed
        val encoderTicks = (-rotations * RobotConfig.TETRIX_TICKS_PER_REVOLUTION).roundToInt()

        addLogLine("Driving for $rotations rotations, which is $encoderTicks ticks")

        val oldMode = robot.leftDriveMotor.mode

        robot.leftDriveMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        robot.leftDriveMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        robot.leftDriveMotor.targetPosition = encoderTicks

        robot.rightDriveMotor.power = if (encoderTicks > 0) 0.5 else -0.5
        robot.leftDriveMotor.power = if (encoderTicks > 0) 0.5 else -0.5
        while (robot.leftDriveMotor.isBusy) {
            addLogLine(robot.leftDriveMotor.currentPosition.toString())
            sleep()
        }
        robot.leftDriveMotor.power = 0.0
        robot.rightDriveMotor.power = 0.0
        robot.leftDriveMotor.mode = oldMode
    }

    private fun turn(robot: RobotConfig, degrees: Int) {
        // SO HACKY DON'T LOOK AT ITTT
        // Encoder ticks are negative because the left drive motor is reversed, but this doesn't
        // change the direction that the encoder counts in
        val encoderTicks = (-degrees * RobotConfig.TETRIX_TICKS_PER_TURN_DEGREE).roundToInt()
        val oldMode = robot.leftDriveMotor.mode

        addLogLine("Moving $degrees degrees, which is $encoderTicks ticks")
        robot.leftDriveMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        robot.leftDriveMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        robot.leftDriveMotor.targetPosition = encoderTicks
        // don't judge
        robot.leftDriveMotor.power = if (encoderTicks >= 0) 0.2 else -0.2
        robot.rightDriveMotor.power = if (encoderTicks >= 0) -0.2 else 0.2
        while (robot.leftDriveMotor.isBusy) {
            addLogLine(robot.leftDriveMotor.currentPosition.toString())
            sleep()
        }
        robot.leftDriveMotor.power = 0.0
        robot.rightDriveMotor.power = 0.0
        robot.leftDriveMotor.mode = oldMode
    }

    override fun runOpMode() {
        // wait for the start button to be pressed.

        waitForStart()
        runtime.reset()
        val robot = RobotConfig(hardwareMap)
        robot.relicElbowServo.position = 1.0

        robot.setGrabbers(RobotConfig.GRABBER_GRABBED)
        robot.lifterMotor.targetPosition = 2 * RobotConfig.TETRIX_TICKS_PER_REVOLUTION
        robot.lifterMotor.power = 0.3
        while (robot.lifterMotor.isBusy) {
            sleep()
        }
        robot.lifterMotor.power = 0.0

        try {
            drive(robot, 1.5) // to safe zone
            robot.lifterMotor.targetPosition = 0
            robot.lifterMotor.power = -0.3
            while (robot.lifterMotor.isBusy) {
                sleep()
            }
            robot.setGrabbers(RobotConfig.GRABBER_RELEASED)
            sleep()
        } catch (e: Exception) {
            addLogLine("HIT EXCEPTION. Stopping op mode.")
            addLogLine("Exception backtrace:")
            addLogLine(e.stackTrace.contentToString())
        }
    }
}