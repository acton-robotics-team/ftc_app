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

import com.disnodeteam.dogecv.CameraViewDisplay
import com.disnodeteam.dogecv.DogeCV
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import edu.spa.ftclib.internal.controller.ErrorTimeThresholdFinishingAlgorithm
import edu.spa.ftclib.internal.controller.FinishableIntegratedController
import edu.spa.ftclib.internal.controller.PIDController
import edu.spa.ftclib.internal.drivetrain.HeadingableTankDrivetrain
import edu.spa.ftclib.internal.sensor.IntegratingGyroscopeSensor
import kotlin.math.roundToInt

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
abstract class BaseAutonomous : LinearOpMode() {
    protected abstract val startLocation: AutonomousStartLocation
    private val runtime = ElapsedTime()

    private enum class GoldPosition {
        RIGHT, LEFT, CENTER
    }

    private fun log(entry: String) {
        // Get caller name
        val stacktrace = Thread.currentThread().stackTrace
        val e = stacktrace[2]
        val methodName = e.methodName

        val logHeader = "[${String.format("%.2f", runtime.seconds())}] [$methodName]"

        telemetry.log().add("$logHeader $entry")
        telemetry.update()
    }

    private fun drive(hw: Hardware, distanceMm: Double, speed: Double = Hardware.DRIVE_FAST) {
        hw.backRightDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        hw.backLeftDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        hw.backRightDrive.mode = DcMotor.RunMode.RUN_TO_POSITION
        hw.backLeftDrive.mode = DcMotor.RunMode.RUN_TO_POSITION

        val requiredEncoderTicks = (distanceMm / 10 * Hardware.DRIVE_ENCODER_TICKS_PER_CM).roundToInt()

        log("Driving $distanceMm mm ($requiredEncoderTicks ticks) to end point")
        val leftEncoderTelemetry = telemetry.addData("Left drive encoder", 0)
        val rightEncoderTelemetry = telemetry.addData("Right drive encoder", 0)
        telemetry.update()

        if (distanceMm > 0) {
            hw.setDrivePower(speed)
        } else {
            hw.setDrivePower(-speed)
        }
        hw.backLeftDrive.targetPosition = requiredEncoderTicks
        hw.backRightDrive.targetPosition = requiredEncoderTicks

        while (opModeIsActive() && hw.backRightDrive.isBusy && hw.backLeftDrive.isBusy) {
            leftEncoderTelemetry.setValue(hw.backLeftDrive.currentPosition)
            rightEncoderTelemetry.setValue(hw.backRightDrive.currentPosition)
            telemetry.update()
            idle()
        }

        hw.setDrivePower(0.0)
        hw.backLeftDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
        hw.backRightDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    private fun doTelemetry(drivetrain: HeadingableTankDrivetrain) {
        val pid = drivetrain.controller.algorithm as PIDController
        telemetry.clear()
        telemetry.addData("heading, target",
                drivetrain.controller.sensorValue.toString() + "," + pid.target)
        telemetry.addData("KP", pid.kp)
        telemetry.addData("KI", pid.ki)
        telemetry.addData("KD", pid.kd)
        telemetry.addData("error", pid.error)
        telemetry.addData("integral", pid.integral)
        telemetry.addData("derivative", pid.derivative)
        telemetry.addData("random", Math.random())
        telemetry.update()
    }

    /**
     * Turns by X degrees (relative)
     */
    private fun turn(hw: Hardware, drivetrain: FourWheelDriveTrain, deg: Float) {
        val rad = deg * Math.PI / 180
        val initHeadingRad = hw.getImuHeading() * Math.PI / 180
        drivetrain.targetHeading = initHeadingRad - rad // magic
        while (drivetrain.isRotating) {
            doTelemetry(drivetrain)
            drivetrain.updateHeading()
            idle()
        }
    }

    private fun turnImprecise(hw: Hardware, deg: Float) {
        val setTurnPower = { power: Double ->
            if (deg > 0) {
                // Turn right (clockwise)
                hw.setRightDrivePower(-power)
                hw.setLeftDrivePower(power)
            } else {
                // Turn left (counterclockwise)
                hw.setRightDrivePower(power)
                hw.setLeftDrivePower(-power)
            }
        }
        val startHeading = hw.getImuHeading()
        val targetHeading = startHeading - deg
        // Drive slowly because reading the IMU is slow and takes a while
        val reachedTargetCondition: (heading: Float) -> Boolean = when {
            deg > 0 -> { heading -> heading < targetHeading }
            else -> { heading -> heading > targetHeading }
        }

        log("Starting turn of $deg degrees from initial heading $startHeading")
        log("Target IMU heading: $targetHeading deg")

        setTurnPower(0.45)

        var heading = hw.getImuHeading()
        val headingTelemetry = telemetry.addData("Current heading", heading)
        while (opModeIsActive() && !reachedTargetCondition(heading)) {
            if (Math.abs(heading - targetHeading) < 10) {
                setTurnPower(Hardware.DRIVE_SLOWEST)
            }
            heading = hw.getImuHeading()
            headingTelemetry.setValue(heading)
            telemetry.update()
        }
        hw.setDrivePower(0.0)
        log("Finished turn.")
    }

    override fun runOpMode() {
        telemetry.isAutoClear = false
        log("Wait for initialization! Do not start!")

        val hw = Hardware(hardwareMap)
        val pid = PIDController(2.5, 0.15, 0.0)
        pid.maxErrorForIntegral = 0.002

        val controller = FinishableIntegratedController(IntegratingGyroscopeSensor(hw.imu), pid, ErrorTimeThresholdFinishingAlgorithm(Math.PI / 12.5, 1.0))
        val drivetrain = FourWheelDriveTrain(hw.backLeftDrive, hw.backRightDrive, hw.frontLeftDrive, hw.frontRightDrive, controller)

        log("Initialized all hardware.")

        val detector = GoldAlignDetector()
        detector.apply {
            // Config as taken from https://github.com/MechanicalMemes/DogeCV/blob/master/Examples/GoldAlignExample.java
            init(hardwareMap.appContext, CameraViewDisplay.getInstance())
            useDefaults()
            alignSize = 100.0
            alignPosOffset = 0.0
            downscale = 0.4
            areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA
            maxAreaScorer.weight = 0.005
            ratioScorer.weight = 5.0
            ratioScorer.perfectRatio = 1.0
            enable()
        }
        log("Initialized DogeCV.")

        telemetry.addData("Status", "Initialized and ready to start!")
        telemetry.update()
        waitForStart()
        runtime.reset()

        // Drop down
        hw.lifter.moveToPosition(Hardware.LIFTER_AUTO_DROP_DOWN_POSITION, 2.5, false)

        // Turn to get out of cage
        turnImprecise(hw, 45f)
        // Back out
        drive(hw, -40.0)

        // Turn until reaching the detector
        hw.setRightDrivePower(0.1)
        hw.setLeftDrivePower(-0.1)
        hw.lifter.apply {
            mode = DcMotor.RunMode.RUN_TO_POSITION
            power = 0.5
            targetPosition = Hardware.LIFTER_AUTO_END_POSITION
        }

        val samplingTimeout = ElapsedTime()
        log("Phase: Gold detection")
        while (!detector.aligned && opModeIsActive() && samplingTimeout.seconds() < 10) {
            telemetry.clearAll()
            telemetry.addData("X pos", detector.xPosition)
            telemetry.update()

            idle()
        }
        // Reached alignment? Maybe or maybe hit 10s timeout
        // We are aligned
        log("Found mineral in " + samplingTimeout.toString())
        log("X position @ found = " + detector.xPosition)
        log("Phase: Gold driving")

        // Always disable the detector
        hw.setDrivePower(0.0)
        detector.disable()

        val heading = hw.getImuHeading()

        val goldPosition = if (heading > -60 && heading < -10) {
            GoldPosition.LEFT
        } else if (heading < 30) {
            GoldPosition.CENTER
        } else {
            GoldPosition.RIGHT
        }
        log("Inferred gold position at $goldPosition")

        when (startLocation) {
            AutonomousStartLocation.FACING_DEPOT -> {
                drive(hw, -825.0) // far enough to always hit the mineral

                // Turn to face the depot
                when (goldPosition) {
                    // turn back to center
                    GoldPosition.LEFT -> turn(hw, drivetrain, hw.getImuHeading())
                    // turn 45 degrees from initial position to aim toward wall
                    GoldPosition.CENTER -> turn(hw, drivetrain, hw.getImuHeading() + 45f)
                    GoldPosition.RIGHT -> {
                        turn(hw, drivetrain, hw.getImuHeading() + 45f)
                        // Drive extra back
                        drive(hw, -450.0)
                    }
                }
                // Reverse into the depot
                drive(hw, -890.0)
                // Release the marker
                hw.markerReleaser.position = Hardware.MARKER_RELEASED
                sleep(1000)
                hw.markerReleaser.position = Hardware.MARKER_RETRACTED
                // Turn toward the crater (enemy side)
                turn(hw, drivetrain, 50f)
                drive(hw, 850.0)
                // Do like a 5 point turn
                turn(hw, drivetrain, -70f)
                drive(hw, 150.0)
                turn(hw, drivetrain, -20f)
                // Drive toward the crater
                drive(hw, 1300.0, 0.7)
            }
            AutonomousStartLocation.FACING_CRATER -> {
                drive(hw, when (goldPosition) {
                    GoldPosition.LEFT -> -660.0
                    GoldPosition.CENTER -> -500.0
                    GoldPosition.RIGHT -> -813.0
                })
                if (goldPosition == GoldPosition.RIGHT || goldPosition == GoldPosition.LEFT) {
                    turn(hw, drivetrain, hw.getImuHeading()) // turn back toward rover
                }
                // Go forward after hitting jewel (back toward lander)
                drive(hw, 300.0) // change the amount as needed
                // Navigate toward depot (turn toward depot) and drive into wall
                turn(hw, drivetrain, hw.getImuHeading() + 85f)
                drive(hw, when (goldPosition) {
                    GoldPosition.RIGHT -> 300.0
                    GoldPosition.CENTER -> 900.2
                    GoldPosition.LEFT -> 1200.4
                })
                drive(hw, 375.0)
                turn(hw, drivetrain, hw.getImuHeading() + 45f)
                // Drive until depot and release the object
                drive(hw, 700.6)
                turn(hw, drivetrain, -90f)
                hw.markerReleaser.position = Hardware.MARKER_RELEASED
                sleep(500)
                hw.markerReleaser.position = Hardware.MARKER_RETRACTED
                turn(hw, drivetrain, hw.getImuHeading() - 135f)

                // Navigate back to crater
                drive(hw, 2032.0)
            }
        }
    }
}
