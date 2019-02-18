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

import android.content.Context
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
class TensorflowDetector(private val context: Context, private val telemetry: Telemetry) {

    /**
     * [.vuforia] is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private lateinit var vuforia: VuforiaLocalizer

    /**
     * [.tfod] is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private lateinit var tfod: TFObjectDetector

    private fun inDetectionArea(mineral: Recognition): Boolean {
        // Remove minerals detected above a certain height; they are likely to
        // be in the crater. Also remove ones too far to the right.
        return mineral.top > 400 && mineral.bottom > 500 && mineral.right < 1000
    }

    fun getPosition(): GoldPosition? {
        // Then sort them so that the first mineral is the leftmost one.
        val minerals = tfod.recognitions
                .filter { inDetectionArea(it) }
                .sortedBy { it.left }
        if (minerals.size != 2) {
            return null
        }

        val leftMineral = minerals[0]
        val rightMineral = minerals[1]

        return when {
            leftMineral.label == LABEL_GOLD_MINERAL && rightMineral.label == LABEL_SILVER_MINERAL ->
                GoldPosition.LEFT
            leftMineral.label == LABEL_SILVER_MINERAL && rightMineral.label == LABEL_GOLD_MINERAL ->
                GoldPosition.CENTER
            leftMineral.label == LABEL_SILVER_MINERAL && rightMineral.label == LABEL_SILVER_MINERAL ->
                GoldPosition.RIGHT
            else -> null
        }
    }

    fun activate() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia()

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod()
        } else {
            throw UnsupportedOperationException("Not supported on this phone")
        }
        tfod.activate()
    }

    fun deactivate() {
        tfod.shutdown()
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private fun initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        val parameters = VuforiaLocalizer.Parameters()

        parameters.vuforiaLicenseKey = VUFORIA_KEY
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters)

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private fun initTfod() {
        val tfodMonitorViewId = context.resources.getIdentifier(
                "tfodMonitorViewId", "id", context.packageName)
        val tfodParameters = TFObjectDetector.Parameters(tfodMonitorViewId)
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia)
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL)
    }

    companion object {
        private val TFOD_MODEL_ASSET = "RoverRuckus.tflite"
        private val LABEL_GOLD_MINERAL = "Gold Mineral"
        private val LABEL_SILVER_MINERAL = "Silver Mineral"

        /**
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
        private val VUFORIA_KEY = "AWbfTmn/////AAABmY0xuIe3C0RHvL3XuzRxyEmOT2OekXBSbqN2jot1si3OGBObwWadfitJR/D6Vk8VEBiW0HG2Q8UAEd0//OliF9aWCRmyDJ1mMqKCJZxpZemfT5ELFuWnJIZWUkKyjQfDNe2RIaAh0ermSxF4Bq77IDFirgggdYJoRIyi2Ys7Gl9lD/tSonV8OnldIN/Ove4/MtEBJTKHqjUEjC5U2khV+26AqkeqbxhFTNiIMl0LcmSSfugGhmWFGFtuPtp/+flPBRGoBO+tSl9P2sV4mSUBE/WrpHqB0Jd/tAmeNvbtgQXtZEGYc/9NszwRLVNl9k13vrBcgsiNxs2UY5xAvA4Wb6LN7Yu+tChwc+qBiVKAQe09\n"
    }
}
