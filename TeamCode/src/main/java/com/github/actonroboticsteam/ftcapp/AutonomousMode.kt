/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode

import com.github.actonroboticsteam.ftcapp.OpModeStoppedException
import com.github.actonroboticsteam.ftcapp.RobotConfig
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime

import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer

import java.util.Locale
import java.util.concurrent.FutureTask
import java.util.concurrent.TimeUnit
import java.util.concurrent.TimeoutException

@Autonomous(name = "Autonomous program")
class AutonomousMode : LinearOpMode() {
    private val runtime = ElapsedTime()
    @Volatile private var logStr = ""
    @Synchronized private fun addLogLine(text: String) {
        logStr += "[" + runtime.time(TimeUnit.SECONDS) + "] " + text + "\n"
        telemetry.addLine(logStr)
        telemetry.update()
    }

    private fun sleepSync() {
        if (!opModeIsActive())
            throw OpModeStoppedException()
        else
            idle()
    }

    private fun turnSync(hw: RobotConfig, degrees: Int) {
        val encoderTicks = Math.round(degrees * RobotConfig.TETRIX_TICKS_PER_TURN_DEGREE).toInt()
        addLogLine(String.format(Locale.US,
                "Turning %d degrees, which is %d ticks", degrees, encoderTicks))

        val oldMode = hw.leftDriveMotor.mode

        hw.leftDriveMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        hw.leftDriveMotor.mode = DcMotor.RunMode.RUN_TO_POSITION

        hw.leftDriveMotor.targetPosition = encoderTicks
        hw.leftDriveMotor.power = if (degrees >= 0) 0.2 else -0.2
        hw.rightDriveMotor.power = if (degrees >= 0) -0.2 else -0.2
        while (hw.leftDriveMotor.isBusy) {
            sleepSync()
        }
        hw.leftDriveMotor.power = 0.0
        hw.rightDriveMotor.power = 0.0
        hw.leftDriveMotor.mode = oldMode
    }

    private fun driveSync(hw: RobotConfig, rotations: Double) {
        val encoderTicks = Math.round(rotations * RobotConfig.TETRIX_TICKS_PER_REVOLUTION).toInt()

        addLogLine("Driving for $rotations rotations, which is $encoderTicks ticks")

        val oldMode = hw.leftDriveMotor.mode

        hw.leftDriveMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        hw.leftDriveMotor.mode = DcMotor.RunMode.RUN_TO_POSITION

        hw.leftDriveMotor.targetPosition = encoderTicks

        hw.leftDriveMotor.power = if (encoderTicks > 0) 0.2 else -0.2
        hw.rightDriveMotor.power = if (encoderTicks > 0) 0.2 else -0.2
        while (hw.leftDriveMotor.isBusy) {
            sleepSync()
        }
        hw.leftDriveMotor.power = 0.0
        hw.rightDriveMotor.power = 0.0
        hw.leftDriveMotor.mode = oldMode
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
        val hw = RobotConfig(hardwareMap)

        hw.leftGrabberServo.position = RobotConfig.GRABBER_GRABBED
        hw.rightGrabberServo.position = RobotConfig.GRABBER_GRABBED

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
                hw.jewelArmServo.position = RobotConfig.JEWEL_ARM_EXTENDED
                addLogLine("Starting jewel task")
                sleep(2000)

                val direction = if (hw.jewelColorSensor.blue() > 0)
                    -1  // left
                else
                    1 // right
                turnSync(hw, 30 * direction)
                turnSync(hw, -30 * direction)
                hw.jewelArmServo.position = RobotConfig.JEWEL_ARM_HALF_EXTENDED
                sleep(1000)
                null
            }
            // Max 10 sec
            val detectGlyphTask = FutureTask<RelicRecoveryVuMark> { this.detectPictogram() }
            //hw.lifterMotor.setTargetPosition(RobotConfig.LIFTER_TOP_LIMIT / 2);
            //hw.lifterMotor.setPower(-0.2);
            jewelTask.run()
            detectGlyphTask.run()

            // Get (blocking) glyph column
            var correctGlyphColumn: RelicRecoveryVuMark
            try {
                correctGlyphColumn = detectGlyphTask.get(10, TimeUnit.SECONDS)
            } catch (e: TimeoutException) {
                addLogLine("Failed to find glyph")
                correctGlyphColumn = RelicRecoveryVuMark.UNKNOWN
            }

            addLogLine("Got glyph column " + correctGlyphColumn)

            // wait for jewel task to finish
            jewelTask.get()
            addLogLine("Jewel task finished")

            // reeeveerse
            hw.leftDriveMotor.power = -0.1
            hw.rightDriveMotor.power = -0.1
            // Detect with ODS
            val columnsToPass = when (correctGlyphColumn) {
                RelicRecoveryVuMark.LEFT -> 3
                RelicRecoveryVuMark.RIGHT -> 2
                RelicRecoveryVuMark.CENTER -> 1
                RelicRecoveryVuMark.UNKNOWN -> 2
            }
            addLogLine("Must pass columns: " + columnsToPass)
            var columnsPassed = 0
            while (columnsToPass > columnsPassed) {
                while (hw.ods.lightDetected == 0.0) {
                    sleepSync()
                }
                // we have hit a glyph column wall
                columnsPassed += 1
                addLogLine("Reached glyph column")
                while (hw.ods.lightDetected > 0) {
                    sleepSync()
                }
            }

            // Now we are at the required column. Turn & move forward until ODS reads
            hw.jewelArmServo.position = RobotConfig.JEWEL_ARM_RETRACTED
            turnSync(hw, -90)
            driveSync(hw, 0.2)
            // Hopefully we've hit the box by now. Release the box!
            hw.rightGrabberServo.position = RobotConfig.GRABBER_RELEASED
            hw.leftGrabberServo.position = RobotConfig.GRABBER_RELEASED
        } catch (e: Exception) {
            addLogLine("Stopping op mode... " + e)
        }

    }
}