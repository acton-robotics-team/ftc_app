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
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.navigation.*
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

    private fun getCurrentLocation(trackables: VuforiaTrackables): OpenGLMatrix? {
        telemetry.clearAll()
        telemetry.addLine("Looking for current location...")
        while (opModeIsActive()) {
            for (trackable in trackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */
                telemetry.addData(trackable.name,
                        if ((trackable.listener as VuforiaTrackableDefaultListener).isVisible) {
                            "Visible"
                        } else {
                            "Not Visible"
                        })

                val robotLocationTransform =
                        (trackable.listener as VuforiaTrackableDefaultListener).updatedRobotLocation
                if (robotLocationTransform != null) {
                    return robotLocationTransform
                }
            }
            telemetry.update()
        }
        return null
    }

    private fun navigateToPoint(hw: Hardware, trackables: VuforiaTrackables, targetXMm: Float, targetYMm: Float) {
        val location = getCurrentLocation(trackables)
                ?: return // pog this shouldn't happen unless opmode is stopped
        // blocks

        telemetry.addData("Current location", location.formatAsTransform())
        telemetry.addLine("Target point: ($targetXMm mm, $targetYMm mm)")
        telemetry.update()

        val xMm = location.translation[0]
        val yMm = location.translation[1]
        val zMm = location.translation[2]

        telemetry.addData("Pos (mm)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                xMm, yMm, zMm)

        // express the rotation of the robot in degrees.
        val vuforiaHeadingDeg = Orientation.getOrientation(
                location,
                AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle

        val desiredHeadingDeg = calculateHeading(xMm, xMm, targetXMm, targetYMm)
        telemetry.addData("Desired heading (deg)", desiredHeadingDeg)

        val startImuHeadingDeg = hw.getImuHeading()
        telemetry.addData("Starting IMU heading (deg)", startImuHeadingDeg)
        val imuHeadingTelemetry = telemetry.addData("Current IMU heading (deg)", startImuHeadingDeg)
        telemetry.update()

        val turnDeg = vuforiaHeadingDeg - desiredHeadingDeg
        val endImuHeadingDeg = calculateImuHeading(startImuHeadingDeg, turnDeg)

        when {
            turnDeg > 0 -> {
                log("Turning counterclockwise to point toward target point")
                hw.setLeftDrivePower(-Hardware.DRIVE_SLOWEST)
                hw.setRightDrivePower(Hardware.DRIVE_SLOWEST)

            }
            turnDeg < 0 -> {
                log("Turning clockwise to point toward target point")
                hw.setLeftDrivePower(Hardware.DRIVE_SLOWEST)
                hw.setRightDrivePower(-Hardware.DRIVE_SLOWEST)
            }
        }
        log("Waiting to reach correct heading")
        while (opModeIsActive()) {
            val currentImuHeadingDeg = hw.getImuHeading()
            imuHeadingTelemetry.setValue(currentImuHeadingDeg)

            if (Math.abs(currentImuHeadingDeg - endImuHeadingDeg) < 10) {
                log("Reached within ten degrees of correct heading")
                break
            } else {
                idle()
            }
        }

        // Drive toward point until encoders read the distance traveled
        hw.setDrivePower(0.0)
        val distanceMm = distanceTo(xMm, yMm, targetXMm, targetYMm)
        drive(hw, distanceMm.toDouble())
    }

    private fun drive(hw: Hardware, distanceMm: Double) {
        hw.backRightDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        hw.backLeftDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        hw.backRightDrive.mode = DcMotor.RunMode.RUN_TO_POSITION
        hw.backLeftDrive.mode = DcMotor.RunMode.RUN_TO_POSITION

        val requiredEncoderTicks = (distanceMm / 10 * Hardware.DRIVE_ENCODER_TICKS_PER_CM).roundToInt()

        log("Driving $distanceMm mm ($requiredEncoderTicks ticks) to end point")
        val leftEncoderTelemetry = telemetry.addData("Left drive encoder", 0)
        val rightEncoderTelemetry = telemetry.addData("Right drive encoder", 0)
        telemetry.update()

        hw.setDrivePower(Hardware.DRIVE_SLOW)
        hw.backLeftDrive.targetPosition = requiredEncoderTicks
        hw.backRightDrive.targetPosition = requiredEncoderTicks

        while (hw.backRightDrive.isBusy || hw.backLeftDrive.isBusy) {
            leftEncoderTelemetry.setValue(hw.backLeftDrive.currentPosition)
            rightEncoderTelemetry.setValue(hw.backRightDrive.currentPosition)
            telemetry.update()
            idle()
        }

        hw.setDrivePower(0.0)
        hw.backLeftDrive.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        hw.backRightDrive.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    /**
     * Turns by X degrees (relative)
     */
    private fun turn(hw: Hardware, deg: Float) {
        val startHeading = hw.getImuHeading()
        log("Starting turn of $deg degrees from initial heading $startHeading")
        val targetHeading = calculateImuHeading(startHeading, deg)
        log("Target IMU heading: $targetHeading deg")

        // Drive slowly because reading the IMU is slow and takes a while
        if (deg > 0) {
            // Turn right (clockwise)
            hw.setRightDrivePower(-Hardware.DRIVE_SLOWEST)
            hw.setLeftDrivePower(Hardware.DRIVE_SLOWEST)
        } else {
            // Turn left (counterclockwise)
            hw.setRightDrivePower(Hardware.DRIVE_SLOWEST)
            hw.setLeftDrivePower(-Hardware.DRIVE_SLOWEST)
        }

        var heading = hw.getImuHeading()
        val headingTelemetry = telemetry.addData("Current heading", heading)
        while (opModeIsActive() && Math.abs(heading - targetHeading) > 10) {
            heading = hw.getImuHeading()
            headingTelemetry.setValue(heading)
            idle()
        }
        log("Finished turn.")
    }

    override fun runOpMode() {
        telemetry.isAutoClear = false
        log("Wait for initialization! Do not start!")

        val hw = Hardware(hardwareMap)

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

//        val vuforia = createVuforia(hardwareMap, hw)
        log("Created Vuforia.")

//        val trackables = configureVuforiaTrackables(hw, vuforia)
        log("Initialized Vuforia trackables.")

        telemetry.addData("Status", "Initialized and ready to start!")
        telemetry.update()
        waitForStart()
        runtime.reset()

        turn(hw, 180f)
        return

        // Lock in grabbers
        hw.grabber.position = Hardware.GRABBER_GRABBED
        hw.lifter.moveToPosition(Hardware.LIFTER_AUTO_DROP_DOWN_POSITION, 0.5, false)

        // Turn to get out of cage
        hw.setRightDrivePower(-Hardware.DRIVE_SLOW)
        hw.setLeftDrivePower(Hardware.DRIVE_SLOW)

        // Back out
        hw.setDrivePower(-Hardware.DRIVE_SLOWEST)

        sleep(500)

        // Turn until reaching the detector
        hw.setRightDrivePower(Hardware.DRIVE_SLOWEST)
        hw.setLeftDrivePower(-Hardware.DRIVE_SLOWEST)

        val samplingTimeout = ElapsedTime()
        while (!detector.aligned && opModeIsActive() && samplingTimeout.seconds() < 10) {
            telemetry.clearAll()
            log("Phase: Gold detection")
            telemetry.addData("X pos", detector.xPosition)
            telemetry.update()

            idle()
        }
        // Reached alignment? Maybe or maybe hit 10s timeout
        if (detector.aligned) {
            // We are aligned
            log("Phase: Gold driving")

            hw.setDrivePower(-Hardware.DRIVE_SLOWEST)

            sleep(2000)
        }

        hw.setDrivePower(0.0)

        // Always disable the detector
        detector.disable()

        if (startLocation == AutonomousStartLocation.FACING_DEPOT) {
            // Navigate toward the depot
            hw.setDrivePower(-Hardware.DRIVE_SLOW)
            sleep(500)
            hw.setDrivePower(0.0)
            turn(hw, 180f) // turn around

            // Extend the arm
//            hw.arm.moveToPosition(Hardware.ARM_HALF_UP, 0.5, true)
//            hw.armExtender.moveToPosition(Hardware.ARM_EXTENDED, 0.5, true)
//
//            // Open the claww for a second
//            hw.rightGrabber.position = Hardware.GRABBER_RELEASED
//            hw.leftGrabber.position = Hardware.GRABBER_RELEASED
//            sleep(100)
//            hw.rightGrabber.position = Hardware.GRABBER_GRABBED
//            hw.leftGrabber.position = Hardware.GRABBER_GRABBED
//
//            // lit, retract the machinery and keep going
//            hw.armExtender.moveToPosition(Hardware.ARM_RETRACTED, 0.5, false)
//            hw.arm.power = 0.0

            // Drive to crater
            // TODO add
//            navigateToPoint(hw, trackables, 1f, 1f)
        }

        log("Done, retracting lifter. Good luck on manual!")

        // Reset everything to normal positions
        hw.lifter.moveToPosition(Hardware.LIFTER_AUTO_END_POSITION, 0.5, false)
    }
}
