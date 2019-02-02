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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
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
abstract class BaseAutonomousWithArm : LinearOpMode() {
    protected abstract val startLocation: AutonomousStartLocation
    private val runtime = ElapsedTime()

    private fun log(entry: String) {
        val logHeader = "[${String.format("%.2f", runtime.seconds())}]"

        telemetry.logEx("$logHeader $entry")
    }

    private fun detectMineral(tf: TensorflowDetector): GoldPosition {
        // Search for mineral with Tensorflow
        val samplingTimeout = ElapsedTime()
        var goldPosition: GoldPosition? = null
        while (opModeIsActive() && goldPosition == null && samplingTimeout.seconds() <= 10) {
            goldPosition = tf.getPosition()
        }

        telemetry.logEx("Found (preliminary) $goldPosition")

        // Spend at least three seconds detecting (sometimes Tensorflow does
        // not detect all minerals in the beginning)
        val timeSpentDetecting = ElapsedTime()
        while (opModeIsActive() && timeSpentDetecting.seconds() <= 2) {
            goldPosition = tf.getPosition()
        }

        if (goldPosition == null) {
            // Uh oh, we didn't manage to identify the mineral in time.
            // As a fallback, just go toward the center one.
            telemetry.logEx("Not found; had to fall back to CENTER mineral.")
            goldPosition = GoldPosition.CENTER
        }
        telemetry.logEx("Got gold position $goldPosition")
        return goldPosition
    }

    override fun runOpMode() {
        telemetry.isAutoClear = false
        log("Wait for initialization! Do not start!")

        val hw = Hardware(hardwareMap, this)
        hw.setHingeServoPosition(1.0)
        log("Initialized all hardware.")

        val tf = TensorflowDetector(hardwareMap.appContext, telemetry)
        tf.activate()
        log("Initialized Tensorflow.")

        // Not using waitforStart because of bug https://github.com/ftctechnh/ftc_app/wiki/Troubleshooting#motorola-e4-phones-disconnecting-momentarily-reported-102018
        while (!opModeIsActive() && !isStopRequested) {
            telemetry.addData("status", "waiting for start command...")
            telemetry.update()
        }
        runtime.reset()

        val goldPosition = detectMineral(tf)
        tf.deactivate()

        // Drop down
        hw.lifter.apply {
            mode = DcMotor.RunMode.RUN_TO_POSITION
            power = 1.0
            targetPosition = Hardware.LIFTER_AUTO_DROP_DOWN_POSITION
        }
        while (opModeIsActive() && hw.lifter.isBusy) {
        }

        // Drop off marker if facing depot
        // This will be done later if facing crater
        if (startLocation == AutonomousStartLocation.FACING_DEPOT) {
            hw.turnFromStart(180f) // Turn toward the depot
            hw.rotateArmFromStartPosition(150f)
            hw.armExtender.apply {
                mode = DcMotor.RunMode.RUN_TO_POSITION
                power = 1.0
                targetPosition = 1120

                while (opModeIsActive() && isBusy) {
                }
            }

            hw.setHingeServoPosition(0.0) // release team marker
            sleep(250)

            hw.armExtender.targetPosition = 0 // Retract le arm
            while (opModeIsActive() && hw.armExtender.isBusy) {
            }
        }

        // Point toward the proper mineral
        hw.turnFromStart(180f + when (goldPosition) {
            GoldPosition.RIGHT -> 45f
            GoldPosition.CENTER -> 0f
            GoldPosition.LEFT -> -45f
        })

        // Begin retracting the lifter
        hw.lifter.targetPosition = Hardware.LIFTER_AUTO_END_POSITION

        // Move arm to the extended position for sampling
        hw.rotateArmFromStartPosition(150f)
        return
        // Hit the proper mineral
        hw.armExtender.apply {
            mode = DcMotor.RunMode.RUN_TO_POSITION
            power = 0.5
            targetPosition = 1120 // TODO find actual position to hit mineral

            while (opModeIsActive() && isBusy) {
            }

            // Okay, we must've hit the mineral now. Retract.
            targetPosition = 0

            while (opModeIsActive() && isBusy) {
            }
        }
        hw.rotateArmFromStartPosition(0f) // Retract arm

        when (startLocation) {
            AutonomousStartLocation.FACING_DEPOT -> {
                // Turn toward the crater (enemy side)
                hw.turn(-130f)
                hw.drive(33.5)
                // Do like a 5 point turn
                hw.turnFromStart(65f)
                hw.drive(5.9)
                hw.turnFromStart(50f)
                // Drive toward the crater
                hw.drive(51.2)
            }
            AutonomousStartLocation.FACING_CRATER -> {
                // Navigate toward depot (turn toward depot) and drive into wall
                hw.turnFromStart(85f)
                // TODO Remeasure
                hw.drive(53.0)
                // Turn toward the depot
                hw.turnFromStart(45f)
                // Drive until depot and release the object
                hw.drive(27.6)
                // Release the arm
                hw.rotateArmFromStartPosition(150f)
                hw.armExtender.apply {
                    mode = DcMotor.RunMode.RUN_TO_POSITION
                    power = 1.0
                    targetPosition = 1120

                    while (opModeIsActive() && isBusy) {
                    }
                }

                hw.setHingeServoPosition(0.0) // release team marker
                sleep(250)
                hw.armExtender.targetPosition = 0 // Retract le arm
                while (opModeIsActive() && hw.armExtender.isBusy) {
                }
                hw.rotateArmFromStartPosition(0f) // Put arm back ty

                // Turn toward crater
                hw.turnFromStart(-135f)

                // Navigate back to crater
                hw.drive(80.0)
            }
        }
    }
}
