package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.navigation.*

const val MM_PER_INCH = 25.4f
const val FTC_FIELD_WIDTH_MM = 12 * 6 * MM_PER_INCH
const val TARGET_HEIGHT_MM = 6 * MM_PER_INCH

const val CAMERA_FORWARD_DISPLACEMENT_MM = 1180   // eg: Camera is 110 mm in front of robot center
const val CAMERA_VERTICAL_DISPLACEMENT_MM = 218   // eg: Camera is 200 mm above ground
const val CAMERA_LEFT_DISPLACEMENT_MM = -30     // eg: Camera is ON the robot's center line

fun createVuforia(hardwareMap: HardwareMap, hw: Hardware): VuforiaLocalizer {
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

fun configureVuforiaTrackables(hw: Hardware, vuforia: VuforiaLocalizer): Map<VuforiaTrackables, VuforiaTrackable> {
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