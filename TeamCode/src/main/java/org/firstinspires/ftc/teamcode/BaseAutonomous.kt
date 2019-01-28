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

    private fun log(entry: String) {
        // Get caller name
        val stacktrace = Thread.currentThread().stackTrace
        val e = stacktrace[2]
        val methodName = e.methodName

        val logHeader = "[${String.format("%.2f", runtime.seconds())}] [$methodName]"

        telemetry.log().add("$logHeader $entry")
        telemetry.update()
    }

    private fun drive(hw: Hardware, inches: Double, speed: Double = Hardware.DRIVE_FAST) {
        hw.backRightDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        hw.backLeftDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        hw.backRightDrive.mode = DcMotor.RunMode.RUN_TO_POSITION
        hw.backLeftDrive.mode = DcMotor.RunMode.RUN_TO_POSITION

        val requiredEncoderTicks = (inches * Hardware.DRIVE_ENCODER_TICKS_PER_IN).roundToInt()

        log("Driving $inches in ($requiredEncoderTicks ticks) to end point")
        val leftEncoderTelemetry = telemetry.addData("Left drive encoder", 0)
        val rightEncoderTelemetry = telemetry.addData("Right drive encoder", 0)
        telemetry.update()

        if (inches > 0) {
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
     * Turns by X degrees relative to the robot's current heading
     */
    private fun turn(hw: Hardware, drivetrain: FourWheelDriveTrain, deg: Float) {
        val rad = deg * Math.PI / 180
        val initHeadingRad = hw.getImuHeading() * Math.PI / 180
        drivetrain.targetHeading = initHeadingRad - rad // magic
        while (opModeIsActive() && drivetrain.isRotating) {
            doTelemetry(drivetrain)
            drivetrain.updateHeading()
        }
    }

    /**
     * Turns from X degrees relative to the starting heading
     */
    private fun turnFromStartPosition(hw: Hardware, drivetrain: FourWheelDriveTrain, deg: Float) {
        turn(hw, drivetrain, hw.getImuHeading() + 90f + deg)
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

        // Not using waitforStart because of bug https://github.com/ftctechnh/ftc_app/wiki/Troubleshooting#motorola-e4-phones-disconnecting-momentarily-reported-102018
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...")
            telemetry.update()
        }
        runtime.reset()

        // Drop down
        hw.lifter.apply {
            mode = DcMotor.RunMode.RUN_TO_POSITION
            power = 1.0
            targetPosition = Hardware.LIFTER_AUTO_DROP_DOWN_POSITION
        }
        while (opModeIsActive() && hw.lifter.isBusy) {}

        // Turn to get out of cage
        turn(hw, drivetrain, 150f)

        // Turn until reaching the detector
        hw.lifter.targetPosition = Hardware.LIFTER_AUTO_END_POSITION

        hw.setRightDrivePower(0.1)
        hw.setLeftDrivePower(-0.1)

        val samplingTimeout = ElapsedTime()
        log("Phase: Gold detection")
        while (!detector.aligned && opModeIsActive() && hw.getImuHeading() < 45f) {
            telemetry.clearAll()
            telemetry.addData("X pos", detector.xPosition)
            telemetry.addData("Lifter position", hw.lifter.currentPosition)
            telemetry.update()
        }
        // Reached alignment? Maybe or maybe hit 10s timeout
        // We are aligned
        log("Found mineral in $samplingTimeout")
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
                if (goldPosition != GoldPosition.RIGHT) {
                    turnImprecise(hw, 7f)
                }
                drive(hw, -32.5) // far enough to always hit the mineral
                
                // Turn to face the depot
                when (goldPosition) {
                    // turn back to center
                    GoldPosition.LEFT -> turnFromStartPosition(hw, drivetrain, 0f)
                    // turn 45 degrees from initial position to aim toward wall
                    GoldPosition.CENTER -> turnFromStartPosition(hw, drivetrain, 45f)
                    GoldPosition.RIGHT -> {
                        turnFromStartPosition(hw, drivetrain, 45f)
                        // Drive extra back
                        drive(hw, -17.7)
                    }
                }
                // Reverse into the depot
                drive(hw, -35.0)
                // Release the marker
                hw.markerReleaser.position = Hardware.MARKER_RELEASED
                sleep(250)
                hw.markerReleaser.position = Hardware.MARKER_RETRACTED
                // Turn toward the crater (enemy side)
                turn(hw, drivetrain, 50f)
                drive(hw, 33.5)
                // Do like a 5 point turn
                turnFromStartPosition(hw, drivetrain, 65f)
                drive(hw, 5.9)
                turnFromStartPosition(hw, drivetrain, 50f)
                // Drive toward the crater
                drive(hw, 51.2)
            }
            AutonomousStartLocation.FACING_CRATER -> {
                drive(hw, when (goldPosition) {
                    GoldPosition.LEFT -> -26.0
                    GoldPosition.CENTER -> -23.6
                    GoldPosition.RIGHT -> -32.0
                })
                if (goldPosition == GoldPosition.RIGHT || goldPosition == GoldPosition.LEFT) {
                    turnFromStartPosition(hw, drivetrain, 0f) // turn back toward rover
                }
                // Go forward after hitting jewel (back toward lander)
                drive(hw, 10.0) // change the amount as needed
                // Navigate toward depot (turn toward depot) and drive into wall
                turnFromStartPosition(hw, drivetrain, 85f)
                drive(hw, when (goldPosition) {
                    GoldPosition.RIGHT -> 11.8
                    GoldPosition.CENTER -> 38.2
                    GoldPosition.LEFT -> 47.25
                })
                drive(hw, 14.8)
                turnFromStartPosition(hw, drivetrain, 45f)
                // Drive until depot and release the object
                drive(hw, 27.6)
                turnImprecise(hw, -90f)
                hw.markerReleaser.position = Hardware.MARKER_RELEASED
                sleep(500)
                hw.markerReleaser.position = Hardware.MARKER_RETRACTED
                turnFromStartPosition(hw, drivetrain, -135f)

                // Navigate back to crater
                drive(hw, 80.0)
            }
        }
    }
}
