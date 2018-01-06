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
        val encoderTicks = (-rotations * RobotConfig.TETRIX_TICKS_PER_REVOLUTION).roundToInt()

        addLogLine("Driving for $rotations rotations, which is $encoderTicks ticks")

        val oldMode = robot.leftDriveMotor.mode

        robot.leftDriveMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        robot.leftDriveMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        robot.leftDriveMotor.targetPosition = encoderTicks


        robot.rightDriveMotor.power = if (encoderTicks > 0) 0.5 else -0.5
    //    sleep(200)
        robot.leftDriveMotor.power = if (encoderTicks > 0) 0.5 else -0.5
        while (robot.leftDriveMotor.isBusy) {
            sleep()
        }
        robot.leftDriveMotor.power = 0.0
        robot.rightDriveMotor.power = 0.0
        robot.leftDriveMotor.mode = oldMode
    }

    private fun turn(robot: RobotConfig, degrees: Int) {
        // Encoder ticks are negative because the left drive motor is reversed, but this doesn't
        // change the direction that the encoder counts in
        val encoderTicks = (degrees * RobotConfig.TETRIX_TICKS_PER_TURN_DEGREE * -1).roundToInt()
        val oldMode = robot.leftDriveMotor.mode

        addLogLine("Moving $degrees degrees, which is $encoderTicks ticks")
        robot.leftDriveMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        robot.leftDriveMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        robot.leftDriveMotor.targetPosition = encoderTicks
        robot.leftDriveMotor.power = if (degrees >= 0) 0.2 else -0.2
        robot.rightDriveMotor.power = if (degrees >= 0) -0.2 else 0.2
        while (robot.leftDriveMotor.isBusy) {
            sleep()
        }
        robot.leftDriveMotor.power = 0.0
        robot.rightDriveMotor.power = 0.0
        robot.leftDriveMotor.mode = oldMode
    }

    private fun detectJewel(): JewelColorDetector.Analysis {
        val params = VuforiaParameters.Builder()
                .cameraDirection(VuforiaLocalizer.CameraDirection.BACK)
                .licenseKey("AcdD/rP/////AAAAGQcYKmwTDk0lulf4t6n2JsQiodu68wCwukVguR/SeZyNkVD0OnUmmSWSrpM2jXTVVNorEhJRyV08URkTRak94XQN8/jPzVxzuOLCQ8VR8uYKuP/JoovnJM2MC3Pc1KvLlrLwWrL4185vpVaQMLRmvCkzNH+lyoEusMC7vwT4ayI6I22ceFumQuAubLp8APiT3omF4KG6W/lqNyJukt9YHgBYO/JJRVPfZg04LEhwFMixYOXfh+moWdf8zCMj+V7GUfH7Q7OGM0jobzVrg0uYboA2nrJBRjQS6j2eGoXX4yRwhmeVLVtBuklgw+n3qXgQ+OX9Lp48xNIApOByAlAhU117gDYYwE5NQ8ADKvtgupKd")
                .build()
        val vuforia = VuforiaNavigation(params)
        val colorDetector = JewelColorDetector(PhoneOrientation.LANDSCAPE)

        while (true) {
            colorDetector.process(vuforia.image)
            val analysis = colorDetector.analysis
            if ((analysis.leftJewelColor != null && analysis.rightJewelColor != null) || runtime.seconds() > 10) {
                return analysis
            } else {
                sleep()
            }
        }
    }

    override fun runOpMode() {
        val robot = RobotConfig(hardwareMap)

        robot.setGrabbers(RobotConfig.GRABBER_GRABBED)

        // wait for the start button to be pressed.
        waitForStart()

        runtime.reset()

        // 1. Hit jewel & scan pictogram simultaneously
        // 2. Move backward and detect cryptobox columns with ODS
        // 3. Turn and move forward to put glyph in cryptobox
        // 4. Release glyph

        try {
            drive(robot, 2.5) // to safe zone
            turn(robot, 90) // turn to cryptobox
            drive(robot, 0.2) // drive into cryptobox
            robot.setGrabbers(RobotConfig.GRABBER_RELEASED) // releasify
        } catch (e: Exception) {
            addLogLine("HIT EXCEPTION. Stopping op mode.")
            addLogLine("Exception backtrace:")
            addLogLine(e.stackTrace.contentToString())
        }
    }
}