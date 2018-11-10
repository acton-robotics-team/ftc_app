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
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.navigation.*
import kotlin.math.roundToInt

const val MM_PER_INCH = 25.4f
const val FTC_FIELD_WIDTH_MM = 12 * 6 * MM_PER_INCH
const val TARGET_HEIGHT_MM = 6 * MM_PER_INCH

const val CAMERA_FORWARD_DISPLACEMENT_MM = 1180   // eg: Camera is 110 mm in front of robot center
const val CAMERA_VERTICAL_DISPLACEMENT_MM = 218   // eg: Camera is 200 mm above ground
const val CAMERA_LEFT_DISPLACEMENT_MM = -30     // eg: Camera is ON the robot's center line

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

    private fun createVuforia(hw: Hardware): VuforiaLocalizer {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        val params = VuforiaLocalizer.Parameters(cameraMonitorViewId)
        params.apply {
            vuforiaLicenseKey = "AWbfTmn/////AAABmY0xuIe3C0RHvL3XuzRxyEmOT2OekXBSbqN2jot1si3OGBObwWadfitJR/D6Vk8VEBiW0HG2Q8UAEd0//OliF9aWCRmyDJ1mMqKCJZxpZemfT5ELFuWnJIZWUkKyjQfDNe2RIaAh0ermSxF4Bq77IDFirgggdYJoRIyi2Ys7Gl9lD/tSonV8OnldIN/Ove4/MtEBJTKHqjUEjC5U2khV+26AqkeqbxhFTNiIMl0LcmSSfugGhmWFGFtuPtp/+flPBRGoBO+tSl9P2sV4mSUBE/WrpHqB0Jd/tAmeNvbtgQXtZEGYc/9NszwRLVNl9k13vrBcgsiNxs2UY5xAvA4Wb6LN7Yu+tChwc+qBiVKAQe09\n"
            cameraName = hw.webcam
        }
        val vuforia = ClassFactory.getInstance().createVuforia(params)
        vuforia.enableConvertFrameToBitmap()

        return vuforia
    }

    private fun configureVuforiaTrackables(hw: Hardware, vuforia: VuforiaLocalizer): Map<VuforiaTrackables, VuforiaTrackable> {
        var allTrackables: Map<VuforiaTrackables, VuforiaTrackable>? = null
        vuforia.apply {
            val targetsRoverRuckus = loadTrackablesFromAsset("RoverRuckus")
            val blueRover = targetsRoverRuckus[0]
            blueRover.name = "Blue-Rover"
            val redFootprint = targetsRoverRuckus[1]
            redFootprint.name = "Red-Footprint"
            val frontCraters = targetsRoverRuckus[2]
            frontCraters.name = "Front-Craters"
            val backSpace = targetsRoverRuckus[3]
            backSpace.name = "Back-Space"

            // For convenience, gather together all the trackable objects in one easily-iterable collection */

            allTrackables = mapOf(
                    VuforiaTrackables.FRONT_CRATERS to frontCraters,
                    VuforiaTrackables.BACK_SPACE to backSpace,
                    VuforiaTrackables.RED_FOOTPRINT to redFootprint,
                    VuforiaTrackables.BLUE_ROVER to blueRover)

            val blueRoverLocationOnField = OpenGLMatrix
                    .translation(0f, FTC_FIELD_WIDTH_MM, TARGET_HEIGHT_MM)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,
                            90.0f, 0.0f, 0.0f))
            blueRover.location = blueRoverLocationOnField

            val redFootprintLocationOnField = OpenGLMatrix
                    .translation(0f, -FTC_FIELD_WIDTH_MM, TARGET_HEIGHT_MM)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,
                            90.0f, 0.0f, 180.0f))
            redFootprint.location = redFootprintLocationOnField

            val frontCratersLocationOnField = OpenGLMatrix
                    .translation(-FTC_FIELD_WIDTH_MM, 0f, TARGET_HEIGHT_MM)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,
                            90.0f, 0.0f, 90.0f))
            frontCraters.location = frontCratersLocationOnField

            val backSpaceLocationOnField = OpenGLMatrix
                    .translation(FTC_FIELD_WIDTH_MM, 0f, TARGET_HEIGHT_MM)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,
                            90.0f, 0.0f, -90.0f))
            backSpace.location = backSpaceLocationOnField

            val webcamLocation = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT_MM.toFloat(),
                            CAMERA_LEFT_DISPLACEMENT_MM.toFloat(),
                            CAMERA_VERTICAL_DISPLACEMENT_MM.toFloat())
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES,
                            -90.0f, 0.0f, 0.0f))

            for ((_, trackable) in allTrackables!!) {
                (trackable.listener as VuforiaTrackableDefaultListener).setCameraLocationOnRobot(hw.webcam, webcamLocation)
            }
            targetsRoverRuckus.activate()
        }
        return allTrackables!!
    }

    private fun getCurrentLocation(trackables: Map<VuforiaTrackables, VuforiaTrackable>): OpenGLMatrix? {
        telemetry.clearAll()
        telemetry.addLine("Looking for current location...")
        while (opModeIsActive()) {
            for ((name, trackable) in trackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */
                telemetry.addData(
                        name.toString(),
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

    private fun navigateToPoint(hw: Hardware, trackables: Map<VuforiaTrackables, VuforiaTrackable>, targetXMm: Float, targetYMm: Float) {
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
                telemetry.log().add("Turning counterclockwise to point toward target point")
                hw.leftDrive.power = -Hardware.DRIVE_SLOWEST
                hw.rightDrive.power = Hardware.DRIVE_SLOWEST

            }
            turnDeg < 0 -> {
                telemetry.log().add("Turning clockwise to point toward target point")
                hw.leftDrive.power = Hardware.DRIVE_SLOWEST
                hw.rightDrive.power = -Hardware.DRIVE_SLOWEST
            }
        }
        telemetry.log().add("Waiting to reach correct heading")
        telemetry.update()
        while (opModeIsActive()) {
            val currentImuHeadingDeg = hw.getImuHeading()
            imuHeadingTelemetry.setValue(currentImuHeadingDeg)

            if (Math.abs(currentImuHeadingDeg - endImuHeadingDeg) < 10) {
                telemetry.log().add("Reached within ten degrees of correct heading")
                telemetry.update()
                break
            } else {
                idle()
            }
        }

        // Drive toward point until encoders read the distance traveled
        hw.leftDrive.power = 0.0
        hw.rightDrive.power = 0.0
        val distanceMm = distanceTo(xMm, yMm, targetXMm, targetYMm)
        drive(hw, distanceMm.toDouble())
    }

    private fun drive(hw: Hardware, distanceMm: Double) {
        hw.leftDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        hw.rightDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        hw.leftDrive.mode = DcMotor.RunMode.RUN_TO_POSITION
        hw.rightDrive.mode = DcMotor.RunMode.RUN_TO_POSITION

        val requiredEncoderTicks = (distanceMm / 10 * Hardware.DRIVE_ENCODER_TICKS_PER_CM).roundToInt()

        telemetry.log().add("Driving $distanceMm mm ($requiredEncoderTicks ticks) to end point")
        val leftEncoderTelemetry = telemetry.addData("Left drive encoder", 0)
        val rightEncoderTelemetry = telemetry.addData("Right drive encoder", 0)
        telemetry.update()

        hw.leftDrive.targetPosition = requiredEncoderTicks
        hw.rightDrive.targetPosition = requiredEncoderTicks
        hw.leftDrive.power = Hardware.DRIVE_SLOW
        hw.rightDrive.power = Hardware.DRIVE_SLOW

        while (hw.leftDrive.isBusy || hw.rightDrive.isBusy) {
            leftEncoderTelemetry.setValue(hw.leftDrive.currentPosition)
            rightEncoderTelemetry.setValue(hw.rightDrive.currentPosition)
            telemetry.update()
            idle()
        }

        hw.leftDrive.power = 0.0
        hw.rightDrive.power = 0.0
        hw.leftDrive.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        hw.rightDrive.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    override fun runOpMode() {
        telemetry.isAutoClear = false
        telemetry.addLine("Wait for initialization! Do not start!")
        telemetry.update()

        val hw = Hardware(hardwareMap)

        telemetry.addLine("Initialized all hardware.")
        telemetry.update()

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

        telemetry.addLine("Initialized DogeCV.")
        telemetry.update()

        val vuforia = createVuforia(hw)
        telemetry.addLine("Created Vuforia.")
        telemetry.update()

        val trackables = configureVuforiaTrackables(hw, vuforia)
        telemetry.addLine("Initialized Vuforia trackables.")
        telemetry.update()

        telemetry.addLine("Started Vuforia with DogeCV integration.")

        telemetry.addData("Status", "Initialized and ready to start!")
        telemetry.update()
        waitForStart()
        runtime.reset()

//        hw.lifter.mode = DcMotor.RunMode.RUN_TO_POSITION
//        // goes from 0 (starting value) down to extend lifter
//        hw.lifter.targetPosition = Hardware.LIFTER_TOP_POSITION
//        hw.lifter.power = 0.5
//        while (hw.lifter.isBusy && opModeIsActive()) {
//            idle()
//        }
//        hw.lifter.power = 0.0
//
//        // Turn to get out of cage
//        hw.rightDrive.power = -Hardware.DRIVE_SLOW
//        hw.leftDrive.power = Hardware.DRIVE_SLOW
//        sleep(1000)
//
//        hw.rightDrive.power = -0.3
//        hw.leftDrive.power = -0.3
//
//        sleep(500)
//
//        // Turn until reaching the detector
//        hw.rightDrive.power = 0.35
//        hw.leftDrive.power = -0.35
//
//        while (!detector.aligned && opModeIsActive()) {
//            telemetry.clearAll()
//            telemetry.addLine("Gold detector phase")
//            telemetry.addData("X pos", detector.xPosition)
//            telemetry.update()
//
//            idle()
//        }
//        // Reached alignment.
//        detector.disable()
//        telemetry.addLine("Gold Driving Phase")
//        telemetry.update()
//
//        hw.leftDrive.power = -0.3
//        hw.rightDrive.power = -0.3
//
//        sleep(2000)
//
//        hw.leftDrive.power = 0.0
//        hw.rightDrive.power = 0.0
//
//
//        telemetry.addLine("Done, retracting lifter. Good luck on manual!")
//        telemetry.update()
//
//        hw.lifter.targetPosition = Hardware.LIFTER_BOTTOM_POSITION
//        hw.lifter.power = -0.5
//        while (hw.lifter.isBusy && opModeIsActive()) {
//            idle()
//        }

//        hw.leftDrive.power = -0.5
//        hw.rightDrive.power = -0.5
//
//        sleep(100)
//
//        hw.leftDrive.power = 0.0
//        hw.rightDrive.power = 0.0
//
//        // Initialize Vuforia tracking phase, then turn to go to depot
//        val trackables = configureVuforiaTrackables()
//        var lastLocation: OpenGLMatrix? = null
//        while (opModeIsActive()) {
//            for (trackable in trackables) {
//                val listener = trackable.listener as VuforiaTrackableDefaultListener
//                if (listener.isVisible) {
//                    telemetry.addData("Visible target", trackable.name)
//                    lastLocation = listener.updatedRobotLocation
//                }
//            }
//
//            if (lastLocation != null) {
//            }
//        }
    }
}
