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
import kotlin.system.measureTimeMillis

@Autonomous(name = "Autonomous program")
class AutonomousMode : LinearOpMode() {
    private val runtime = ElapsedTime()
    @Volatile private var logs = ""
    @Synchronized private fun log(text: String) {
        logs += "[${runtime.time(TimeUnit.SECONDS)}] $text\n"
        telemetry.addLine(logs)
        telemetry.update()
    }

    @Synchronized private fun sleep() = if (!opModeIsActive()) {
        throw OpModeStoppedException()
    } else {
        idle()
    }

    private fun detectPictogram(): RelicRecoveryVuMark {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        val parameters = VuforiaLocalizer.Parameters(cameraMonitorViewId)
        parameters.vuforiaLicenseKey = "AcdD/rP/////AAAAGQcYKmwTDk0lulf4t6n2JsQiodu68wCwukVguR/SeZyNkVD0OnUmmSWSrpM2jXTVVNorEhJRyV08URkTRak94XQN8/jPzVxzuOLCQ8VR8uYKuP/JoovnJM2MC3Pc1KvLlrLwWrL4185vpVaQMLRmvCkzNH+lyoEusMC7vwT4ayI6I22ceFumQuAubLp8APiT3omF4KG6W/lqNyJukt9YHgBYO/JJRVPfZg04LEhwFMixYOXfh+moWdf8zCMj+V7GUfH7Q7OGM0jobzVrg0uYboA2nrJBRjQS6j2eGoXX4yRwhmeVLVtBuklgw+n3qXgQ+OX9Lp48xNIApOByAlAhU117gDYYwE5NQ8ADKvtgupKd"
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK
        val vuforia = ClassFactory.createVuforiaLocalizer(parameters)
        val relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark")
        val relicTemplate = relicTrackables[0]
        relicTemplate.name = "relicVuMarkTemplate" // can help in debugging; otherwise not necessary
        relicTrackables.activate()
        while (true) {
            val vuMark = RelicRecoveryVuMark.from(relicTemplate)
            if (vuMark != RelicRecoveryVuMark.UNKNOWN || runtime.seconds() > 10) {
                return vuMark
            } else {
                sleep()
            }
        }
    }

    override fun runOpMode() {
        val robot = RobotConfig(hardwareMap)

        robot.leftGrabberServo.position = RobotConfig.GRABBER_GRABBED
        robot.rightGrabberServo.position = RobotConfig.GRABBER_GRABBED

        // wait for the start button to be pressed.
        waitForStart()

        runtime.reset()

        // 1. Hit jewel & scan pictogram simultaneously
        // 2. Move backward and detect cryptobox columns with ODS
        // 3. Turn and move forward to put glyph in cryptobox
        // 4. Release glyph

        try {
            // TODO: Add moving back and forth if neither detected
            val jewelTask = FutureTask<Void> {
                log("jewel task: lowering jewel arm servo")
                robot.sensorStickServo.position = RobotConfig.JEWEL_ARM_HALF_EXTENDED
                sleep(1000)
                robot.sensorStickServo.position = RobotConfig.JEWEL_ARM_EXTENDED
                sleep( 1000)

                val blueOutput = robot.jewelColorSensor.blue()
                log("jewel task: color sensor reports blue value of $blueOutput")
                val moveAmount = if (blueOutput > 0) {
                    0.1 // forward
                } else {
                    -0.1 // back
                }
                log("Moving $moveAmount and back again")
                drive(robot, moveAmount)
                drive(robot, -moveAmount)
                robot.sensorStickServo.position = RobotConfig.JEWEL_ARM_HALF_EXTENDED
                sleep(1000)
                null
            }
            // Max 10 sec
            robot.lifterMotor.targetPosition = RobotConfig.LIFTER_TOP_LIMIT / 2
            robot.lifterMotor.power = 0.2
            jewelTask.run()

            val correctGlyphColumn = detectPictogram()

            log("Got glyph column " + correctGlyphColumn)

            // wait for jewel task to finish
            while (!jewelTask.isDone) {
                sleep()
            }
            log("Jewel task finished!")

            // reeeveerse
            robot.leftDriveMotor.power = -0.1
            robot.rightDriveMotor.power = -0.1
            // Detect with ODS
            val columnsToPass = when (correctGlyphColumn) {
                RelicRecoveryVuMark.LEFT -> 3
                RelicRecoveryVuMark.RIGHT -> 2
                RelicRecoveryVuMark.CENTER -> 1
                // lmao just guess 2 columns
                RelicRecoveryVuMark.UNKNOWN -> 2
                else -> 2 // lmao
            }
            log("Must pass $columnsToPass columns")
            var columnsPassed = 0
            while (columnsToPass > columnsPassed) {
                while (robot.ods.lightDetected == 0.0) {
                    sleep()
                }
                // we have hit a glyph column wall
                columnsPassed += 1
                log(
                        "Passed $columnsPassed columns, ${columnsToPass - columnsPassed} left")
                while (robot.ods.lightDetected > 0) {
                    sleep()
                }
            }

            // Now we are at the required column. Turn & move forward until ODS reads
            robot.sensorStickServo.position = RobotConfig.JEWEL_ARM_RETRACTED
            turn(robot, degrees = -90)
            drive(robot, rotations = 0.2)
            // Hopefully we've hit the cryptobox by now. Release the glyph!
            robot.rightGrabberServo.position = RobotConfig.GRABBER_RELEASED
            robot.leftGrabberServo.position = RobotConfig.GRABBER_RELEASED
        } catch (e: OpModeStoppedException) {
            log("Stop.")
        } catch (e: Exception) {
            log("HIT EXCEPTION. Stopping op mode.")
            log(e.toString())
            log("Exception backtrace:")
            log(e.stackTrace.contentToString())
        }
    }

    private fun turn(robot: RobotConfig, degrees: Int) {
        // Encoder ticks are negative because the left drive motor is reversed, but this doesn't
        // change the direction that the encoder counts in
        val encoderTicks = (degrees * RobotConfig.TETRIX_TICKS_PER_TURN_DEGREE * -1).roundToInt()
        val oldMode = robot.leftDriveMotor.mode

        robot.leftDriveMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        robot.leftDriveMotor.mode = DcMotor.RunMode.RUN_TO_POSITION

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

    private fun drive(robot: RobotConfig, rotations: Double) {
        // Encoder ticks are negative because the left drive motor is reversed, but this doesn't
        // change the direction that the encoder counts in
        val encoderTicks = (rotations * RobotConfig.TETRIX_TICKS_PER_REVOLUTION * -1).roundToInt()
        val oldMode = robot.leftDriveMotor.mode

        robot.leftDriveMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        robot.leftDriveMotor.mode = DcMotor.RunMode.RUN_TO_POSITION

        robot.leftDriveMotor.targetPosition = encoderTicks

        robot.leftDriveMotor.power = if (rotations > 0) 0.2 else -0.2
        robot.rightDriveMotor.power = if (rotations > 0) 0.2 else -0.2
        while (robot.leftDriveMotor.isBusy) {
            sleep()
        }
        robot.leftDriveMotor.power = 0.0
        robot.rightDriveMotor.power = 0.0
        robot.leftDriveMotor.mode = oldMode
    }
}