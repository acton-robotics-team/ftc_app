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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition

abstract class BaseAutonomous : LinearOpMode() {
    protected abstract val startLocation: AutonomousStartLocation
    private val runtime = ElapsedTime()

    private fun log(entry: String) {
        val logHeader = "[${String.format("%.2f", runtime.seconds())}]"

        telemetry.logEx("$logHeader $entry")
    }

    private fun printMinerals(minerals: List<Recognition>) {
        minerals.forEach {
            log("Mineral: ${it.label} @ bottom ${it.bottom}, top ${it.top}," +
                    "left ${it.left}, right ${it.right}")
        }
    }

    private fun detectMineral(tf: TensorflowDetector): GoldPosition {
        // Search for mineral with Tensorflow
        val samplingTimeout = ElapsedTime()
        var goldPosition: GoldPosition? = null
        while (opModeIsActive() && goldPosition == null && samplingTimeout.seconds() <= 10) {
            goldPosition = tf.getPosition()
        }

        log("Found (preliminary) $goldPosition")

        // Spend at least three seconds detecting (sometimes Tensorflow does
        // not detect all minerals in the beginning)
        val timeSpentDetecting = ElapsedTime()
        while (opModeIsActive() && timeSpentDetecting.seconds() <= 2) {
            goldPosition = tf.getPosition()
        }

        if (goldPosition == null) {
            // Uh oh, we didn't manage to identify the mineral in time.
            // As a fallback, just go toward the center one.
            log("Not found; had to fall back to CENTER mineral.")
            goldPosition = GoldPosition.CENTER
        }
        log("Got gold position $goldPosition")
        printMinerals(tf.getRecognitions())
        return goldPosition
    }

    override fun runOpMode() {
        log("Wait for initialization! Do not start!")

        val hw = Hardware(hardwareMap, this)
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

        // Turn to get out of cage; we are currently sideways
        hw.turnFromStart(0f)
        hw.turn(-270f)

        // Begin retracting the lifter
        hw.lifter.targetPosition = Hardware.LIFTER_AUTO_END_POSITION

        hw.turn(when (goldPosition) {
            GoldPosition.LEFT -> 45f
            GoldPosition.CENTER -> 0f
            GoldPosition.RIGHT -> -45f
        })

        // Hit the gold mineral
        hw.drive(24.0) // far enough to always hit the mineral

        when (startLocation) {
            AutonomousStartLocation.FACING_DEPOT -> {
                // Turn to face the depot
                when (goldPosition) {
                    // turn back to center to drive toward the depot
                    GoldPosition.RIGHT -> hw.turn(-45f)
                    // turn 45 degrees from initial position to aim toward wall
                    GoldPosition.CENTER -> hw.turn(-45f)
                    GoldPosition.LEFT -> {
                        hw.turnFromStart(-90f)
                    }
                }
                // Drive into the depot
                hw.drive(35.0)
                return
                // Release the marker

                hw.markerReleaser.position = Hardware.MARKER_RELEASED
                sleep(250)
                hw.markerReleaser.position = Hardware.MARKER_RETRACTED
                // Turn toward the crater (enemy side)
                hw.turn(130f)
                hw.drive(33.5)
                // Do like a 5 point turn
                hw.turnFromStart(-65f)
                hw.drive(5.9)
                hw.turnFromStart(-50f)
                // Drive toward the crater
                hw.drive(51.2)
            }
            AutonomousStartLocation.FACING_CRATER -> {
                if (goldPosition == GoldPosition.RIGHT || goldPosition == GoldPosition.LEFT) {
                    hw.turnFromStart(0f) // turn back toward rover
                }
                hw.drive(-15.0) // change the amount as needed
                // Navigate toward depot (turn toward depot) and drive into wall
                hw.turnFromStart(-180f)
                hw.drive(when (goldPosition) {
                    GoldPosition.RIGHT -> 11.8
                    GoldPosition.CENTER -> 38.2
                    GoldPosition.LEFT -> 47.25
                })
                hw.drive(14.8)
                hw.turn(45f)
                // Drive until depot and release the object
                hw.drive(27.6)
                hw.turnImprecise(90f)
                hw.rotateArmFromStartPosition(62f)
                hw.boxHingeServo.position = 1.0
                hw.rotateArmFromStartPosition(0f)
                hw.turn(90f)

                // Navigate back to crater
                hw.drive(80.0)
            }
        }
    }
}
