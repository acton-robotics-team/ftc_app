/* Copyright (c) 2018 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.Disabled

import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT

import java.util.ArrayList


/**
 * This 2018-2019 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 *
 * Vuforia uses the phone's webcam to inspect it's surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the webcam.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the webcam.
 *
 * This example assumes a "square" field configuration where the red and blue alliance stations
 * are on opposite walls of each other.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.
 *
 * The four vision targets are located in the center of each of the perimeter walls with
 * the images facing inwards towards the robots:
 * - BlueRover is the Mars Rover image target on the wall closest to the blue alliance
 * - RedFootprint is the Lunar Footprint target on the wall closest to the red alliance
 * - FrontCraters is the Lunar Craters image target on the wall closest to the audience
 * - BackSpace is the Deep Space image target on the wall farthest from the audience
 *
 * A final calculation then uses the location of the webcam on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 *
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@TeleOp(name = "Concept: Vuforia Rover Nav", group = "Concept")
class ConceptVuforiaNavRoverRuckus : LinearOpMode() {

    private var lastLocation: OpenGLMatrix? = null
    private var targetVisible = false

    /**
     * [.vuforia] is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    var vuforia: VuforiaLocalizer? = null

    override fun runOpMode() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a webcam preview resource (on the RC phone);
         * If no webcam monitor is desired, use the parameterless constructor instead (commented out below).
         */
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        val parameters = VuforiaLocalizer.Parameters(cameraMonitorViewId)

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY
        parameters.cameraDirection = CAMERA_CHOICE

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.createVuforiaLocalizer(parameters)

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        val targetsRoverRuckus = vuforia!!.loadTrackablesFromAsset("RoverRuckus")
        val blueRover = targetsRoverRuckus[0]
        blueRover.name = "Blue-Rover"
        val redFootprint = targetsRoverRuckus[1]
        redFootprint.name = "Red-Footprint"
        val frontCraters = targetsRoverRuckus[2]
        frontCraters.name = "Front-Craters"
        val backSpace = targetsRoverRuckus[3]
        backSpace.name = "Back-Space"

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        val allTrackables = ArrayList<VuforiaTrackable>()
        allTrackables.addAll(targetsRoverRuckus)

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of *transformation matrices.*
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See [Transformation Matrix](https://en.wikipedia.org/wiki/Transformation_matrix)
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the [OpenGLMatrix] class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         * - The X axis runs from your left to the right. (positive from the center to the right)
         * - The Y axis runs from the Red Alliance Station towards the other side of the field
         * where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         * - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         * coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        val blueRoverLocationOnField = OpenGLMatrix
                .translation(0f, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90f, 0f, 0f))
        blueRover.location = blueRoverLocationOnField

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         * and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        val redFootprintLocationOnField = OpenGLMatrix
                .translation(0f, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90f, 0f, 180f))
        redFootprint.location = redFootprintLocationOnField

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         * and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        val frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0f, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90f, 0f, 90f))
        frontCraters.location = frontCratersLocationOnField

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         * and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        val backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0f, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90f, 0f, -90f))
        backSpace.location = backSpaceLocationOnField

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * webcam is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) webcam:
         * We need to rotate the webcam around it's long axis to bring the rear webcam forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) webcam
         * We need to rotate the webcam around it's long axis to bring the FRONT webcam forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the webcam lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        val CAMERA_FORWARD_DISPLACEMENT = 110   // eg: Camera is 110 mm in front of robot center
        val CAMERA_VERTICAL_DISPLACEMENT = 200   // eg: Camera is 200 mm above ground
        val CAMERA_LEFT_DISPLACEMENT = 0     // eg: Camera is ON the robot's center line

        val phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT.toFloat(), CAMERA_LEFT_DISPLACEMENT.toFloat(), CAMERA_VERTICAL_DISPLACEMENT.toFloat())
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        (if (CAMERA_CHOICE == FRONT) 90 else -90).toFloat(), 0f, 0f))

        /**  Let all the trackable listeners know where the phone is.   */
        for (trackable in allTrackables) {
            (trackable.listener as VuforiaTrackableDefaultListener).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection)
        }

        /** Wait for the game to begin  */
        telemetry.addData(">", "Press Play to start tracking")
        telemetry.update()
        waitForStart()

        /** Start tracking the data sets we care about.  */
        targetsRoverRuckus.activate()
        while (opModeIsActive()) {

            // check all the trackable target to see which one (if any) is visible.
            targetVisible = false
            for (trackable in allTrackables) {
                if ((trackable.listener as VuforiaTrackableDefaultListener).isVisible) {
                    telemetry.addData("Visible Target", trackable.name)
                    targetVisible = true

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    val robotLocationTransform = (trackable.listener as VuforiaTrackableDefaultListener).updatedRobotLocation
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform
                    }
                    break
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                val translation = lastLocation!!.translation
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch)

                // express the rotation of the robot in degrees.
                val rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES)
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle)
            } else {
                telemetry.addData("Visible Target", "none")
            }
            telemetry.update()
        }
    }

    companion object {

        /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
        private val VUFORIA_KEY = " AcdD/rP/////AAAAGQcYKmwTDk0lulf4t6n2JsQiodu68wCwukVguR/SeZyNkVD0OnUmmSWSrpM2jXTVVNorEhJRyV08URkTRak94XQN8/jPzVxzuOLCQ8VR8uYKuP/JoovnJM2MC3Pc1KvLlrLwWrL4185vpVaQMLRmvCkzNH+lyoEusMC7vwT4ayI6I22ceFumQuAubLp8APiT3omF4KG6W/lqNyJukt9YHgBYO/JJRVPfZg04LEhwFMixYOXfh+moWdf8zCMj+V7GUfH7Q7OGM0jobzVrg0uYboA2nrJBRjQS6j2eGoXX4yRwhmeVLVtBuklgw+n3qXgQ+OX9Lp48xNIApOByAlAhU117gDYYwE5NQ8ADKvtgupKd"

        // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
        // We will define some constants and conversions here
        private val mmPerInch = 25.4f
        private val mmFTCFieldWidth = 12 * 6 * mmPerInch       // the width of the FTC field (from the center point to the outer panels)
        private val mmTargetHeight = 6 * mmPerInch          // the height of the center of the target image above the floor

        // Select which webcam you want use.  The FRONT webcam is the one on the same side as the screen.
        // Valid choices are:  BACK or FRONT
        private val CAMERA_CHOICE = BACK
    }
}
