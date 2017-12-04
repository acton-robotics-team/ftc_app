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

@Autonomous(name = "Autonomous program")
class AutonomousMode : LinearOpMode() {
    private val runtime = ElapsedTime()
    @Volatile private var logStr = ""
    @Synchronized private fun addLogLine(text: String) {
        logStr += "[${runtime.time(TimeUnit.SECONDS)}] $text\n"
        telemetry.addLine(logStr)
        telemetry.update()
    }

    private fun sleepSync() {
        if (!opModeIsActive())
            throw OpModeStoppedException()
        else
            idle()
    }

    private fun turnSync(robot: RobotConfig, degrees: Int) {
        val encoderTicks = (degrees * RobotConfig.TETRIX_TICKS_PER_TURN_DEGREE).roundToInt()
        addLogLine("Turning $degrees degrees, which is $encoderTicks ticks")

        val oldMode = robot.leftDriveMotor.mode

        robot.leftDriveMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        robot.leftDriveMotor.mode = DcMotor.RunMode.RUN_TO_POSITION

        robot.leftDriveMotor.targetPosition = encoderTicks
        robot.leftDriveMotor.power = if (degrees >= 0) 0.2 else -0.2
        robot.rightDriveMotor.power = if (degrees >= 0) -0.2 else -0.2
        while (robot.leftDriveMotor.isBusy) {
            sleepSync()
        }
        robot.leftDriveMotor.power = 0.0
        robot.rightDriveMotor.power = 0.0
        robot.leftDriveMotor.mode = oldMode
    }

    private fun driveSync(robot: RobotConfig, rotations: Double) {
        val encoderTicks = (rotations * RobotConfig.TETRIX_TICKS_PER_REVOLUTION).roundToInt()

        addLogLine("Driving for $rotations rotations, which is $encoderTicks ticks")

        val oldMode = robot.leftDriveMotor.mode

        robot.leftDriveMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        robot.leftDriveMotor.mode = DcMotor.RunMode.RUN_TO_POSITION

        robot.leftDriveMotor.targetPosition = encoderTicks

        robot.leftDriveMotor.power = if (encoderTicks > 0) 0.2 else -0.2
        robot.rightDriveMotor.power = if (encoderTicks > 0) 0.2 else -0.2
        while (robot.leftDriveMotor.isBusy) {
            sleepSync()
        }
        robot.leftDriveMotor.power = 0.0
        robot.rightDriveMotor.power = 0.0
        robot.leftDriveMotor.mode = oldMode
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
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                return vuMark
            } else {
                sleepSync()
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

        // 1. Drive straight into wall
        // 2. Scan pictogram
        // 3. Hit jewel
        // 4. Put block we already have in cryptobox
        // ...

        try {
            // TODO: Add moving back and forth if neither detected
            val jewelTask = FutureTask<Void> {
                robot.jewelArmServo.position = RobotConfig.JEWEL_ARM_EXTENDED
                addLogLine("Starting jewel task")
                sleep(2000)

                val direction = if (robot.jewelColorSensor.blue() > 0)
                    -1  // left
                else
                    1 // right
                turnSync(robot, 30 * direction)
                turnSync(robot, -30 * direction)
                robot.jewelArmServo.position = RobotConfig.JEWEL_ARM_HALF_EXTENDED
                sleep(1000)
                null
            }
            // Max 10 sec
            val detectGlyphTask = FutureTask<RelicRecoveryVuMark> { this.detectPictogram() }
            //robot.lifterMotor.setTargetPosition(RobotConfig.LIFTER_TOP_LIMIT / 2);
            //robot.lifterMotor.setPower(-0.2);
            jewelTask.run()
            detectGlyphTask.run()

            // Get (blocking) glyph column
            val correctGlyphColumn = try {
                detectGlyphTask.get(10, TimeUnit.SECONDS)
            } catch (e: TimeoutException) {
                addLogLine("Failed to find glyph")
                RelicRecoveryVuMark.UNKNOWN
            }

            addLogLine("Got glyph column " + correctGlyphColumn)

            // wait for jewel task to finish
            jewelTask.get()
            addLogLine("Jewel task finished!")

            // reeeveerse
            robot.leftDriveMotor.power = -0.1
            robot.rightDriveMotor.power = -0.1
            // Detect with ODS
            val columnsToPass = when (correctGlyphColumn) {
                RelicRecoveryVuMark.LEFT -> 3
                RelicRecoveryVuMark.RIGHT -> 2
                RelicRecoveryVuMark.CENTER -> 1
                RelicRecoveryVuMark.UNKNOWN -> 2
                else -> 2
            }
            addLogLine("Must pass $columnsToPass columns")
            var columnsPassed = 0
            while (columnsToPass > columnsPassed) {
                while (robot.ods.lightDetected == 0.0) {
                    sleepSync()
                }
                // we have hit a glyph column wall
                columnsPassed += 1
                addLogLine("Reached glyph column")
                while (robot.ods.lightDetected > 0) {
                    sleepSync()
                }
            }

            // Now we are at the required column. Turn & move forward until ODS reads
            robot.jewelArmServo.position = RobotConfig.JEWEL_ARM_RETRACTED
            turnSync(robot, -90)
            driveSync(robot, 0.2)
            // Hopefully we've hit the box by now. Release the box!
            robot.rightGrabberServo.position = RobotConfig.GRABBER_RELEASED
            robot.leftGrabberServo.position = RobotConfig.GRABBER_RELEASED
        } catch (e: Exception) {
            addLogLine("HIT EXCEPTION. Stopping op mode.")
            addLogLine("Exception backtrace:")
            addLogLine(e.stackTrace.contentToString())
        }
    }
}