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
import com.disnodeteam.dogecv.Dogeforia
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.navigation.*
import java.util.Collections.addAll


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
@Autonomous(name = "Autonomous mode")
class AutonomousMode : LinearOpMode() {
    private val runtime = ElapsedTime()

    fun initializeVuforia(): List<VuforiaTrackable> {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        val params = VuforiaLocalizer.Parameters()
        params.apply {
            vuforiaLicenseKey = "AWbfTmn/////AAABmY0xuIe3C0RHvL3XuzRxyEmOT2OekXBSbqN2jot1si3OGBObwWadfitJR/D6Vk8VEBiW0HG2Q8UAEd0//OliF9aWCRmyDJ1mMqKCJZxpZemfT5ELFuWnJIZWUkKyjQfDNe2RIaAh0ermSxF4Bq77IDFirgggdYJoRIyi2Ys7Gl9lD/tSonV8OnldIN/Ove4/MtEBJTKHqjUEjC5U2khV+26AqkeqbxhFTNiIMl0LcmSSfugGhmWFGFtuPtp/+flPBRGoBO+tSl9P2sV4mSUBE/WrpHqB0Jd/tAmeNvbtgQXtZEGYc/9NszwRLVNl9k13vrBcgsiNxs2UY5xAvA4Wb6LN7Yu+tChwc+qBiVKAQe09\n"
            fillCameraMonitorViewParent = true
        }
        val vuforia = Dogeforia(params)
        var allTrackables: List<VuforiaTrackable>? = null
        vuforia.apply {
            enableConvertFrameToBitmap()
            val targetsRoverRuckus = loadTrackablesFromAsset("RoverRuckus")
            val blueRover = targetsRoverRuckus[0]
            blueRover.name = "Blue-Rover"
            val redFootprint = targetsRoverRuckus[1]
            redFootprint.name = "Red-Footprint"
            val frontCraters = targetsRoverRuckus[2]
            frontCraters.name = "Front-Craters"
            val backSpace = targetsRoverRuckus[3]
            backSpace.name = "Back-Space"

            val mmPerInch = 25.4f
            val mmFTCFieldWidth = 12 * 6 * mmPerInch
            val mmTargetHeight = 6 * mmPerInch

            // For convenience, gather together all the trackable objects in one easily-iterable collection */

            allTrackables = targetsRoverRuckus.toList()

            val blueRoverLocationOnField = OpenGLMatrix
                    .translation(0f, mmFTCFieldWidth, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,
                            90.0f, 0.0f, 0.0f))
            blueRover.location = blueRoverLocationOnField

            val redFootprintLocationOnField = OpenGLMatrix
                    .translation(0f, -mmFTCFieldWidth, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,
                            90.0f, 0.0f, 180.0f))
            redFootprint.location = redFootprintLocationOnField

            val frontCratersLocationOnField = OpenGLMatrix
                    .translation(-mmFTCFieldWidth, 0f, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,
                            90.0f, 0.0f, 90.0f))
            frontCraters.location = frontCratersLocationOnField

            val backSpaceLocationOnField = OpenGLMatrix
                    .translation(mmFTCFieldWidth, 0f, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,
                            90.0f, 0.0f, -90.0f))
            backSpace.location = backSpaceLocationOnField

            val CAMERA_FORWARD_DISPLACEMENT = 110   // eg: Camera is 110 mm in front of robot center
            val CAMERA_VERTICAL_DISPLACEMENT = 200   // eg: Camera is 200 mm above ground
            val CAMERA_LEFT_DISPLACEMENT = 0     // eg: Camera is ON the robot's center line
            val CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.BACK

            val phoneLocationOnRobot = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT.toFloat(), CAMERA_LEFT_DISPLACEMENT.toFloat(), CAMERA_VERTICAL_DISPLACEMENT.toFloat())
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES,
                            if (CAMERA_CHOICE === VuforiaLocalizer.CameraDirection.FRONT) 90.0f else -90.0f, 0.0f, 0.0f))

            for (trackable in allTrackables!!) {
                (trackable.listener as VuforiaTrackableDefaultListener).setPhoneInformation(phoneLocationOnRobot, params.cameraDirection)
            }
        }
        return allTrackables!!
    }

    override fun runOpMode() {
        telemetry.addData("Status", "Initialized")
        telemetry.update()

        val hw = Hardware(hardwareMap)

        waitForStart()
        runtime.reset()

        hw.lifter.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        hw.lifter.mode = DcMotor.RunMode.RUN_TO_POSITION
        // goes from 0 (starting value) down to extend lifter
        hw.lifter.targetPosition = -Hardware.LIFTER_TOP_POSITION
        hw.lifter.power = 0.5
        while (hw.lifter.isBusy && opModeIsActive()) {
            idle()
        }
        hw.lifter.power = 0.0
        hw.lifter.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER // set bottom value to 0 value

        // Turn to get out of cage
        hw.rightDrive.power = -Hardware.SLOW_SPEED
        hw.leftDrive.power = Hardware.SLOW_SPEED
        sleep(1000)

        hw.rightDrive.power = 0.0
        hw.leftDrive.power = 0.0

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

        // Turn until reaching the detector
        hw.leftDrive.power = -0.5
        hw.rightDrive.power = 0.5
        while (!detector.aligned && opModeIsActive()) {
            telemetry.addLine("Gold Detector Phase")
            telemetry.addData("X pos", detector.xPosition)
            telemetry.update()

            idle()
        }
        hw.leftDrive.power = 0.5
        hw.rightDrive.power = 0.5

        sleep(100)

        hw.leftDrive.power = 0.0
        hw.rightDrive.power = 0.0

        // Initialize Vuforia tracking phase, then turn to go to depot
        val trackables = initializeVuforia()
        var lastLocation: OpenGLMatrix? = null
        while (opModeIsActive()) {
            for (trackable in trackables) {
                val listener = trackable.listener as VuforiaTrackableDefaultListener
                if (listener.isVisible) {
                    telemetry.addData("Visible target", trackable.name)
                    lastLocation = listener.updatedRobotLocation
                }
            }

            if (lastLocation != null) {
            }
        }
    }
}
