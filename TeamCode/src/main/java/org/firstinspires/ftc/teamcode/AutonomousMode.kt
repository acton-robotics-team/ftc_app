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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime

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
    }
}
