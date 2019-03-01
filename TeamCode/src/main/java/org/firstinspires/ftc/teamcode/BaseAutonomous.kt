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

    private fun printMinerals(minerals: List<Recognition>) = minerals.forEach {
        log("Mineral: ${it.label} @ bottom ${it.bottom}, top ${it.top}," +
                "left ${it.left}, right ${it.right}")
    }

    private fun detectMineral(tf: TensorflowDetector): GoldPosition {
        // Search for mineral with Tensorflow
        val goldPosition = tf.getPosition()
        log("Found $goldPosition")
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
        hw.boxGate.position = 1.0

        // Not using waitforStart because of bug https://github.com/ftctechnh/ftc_app/wiki/Troubleshooting#motorola-e4-phones-disconnecting-momentarily-reported-102018
        while (!opModeIsActive() && !isStopRequested) {
            telemetry.addData("Status", "Waiting for start command...")
            telemetry.addData("Detected gold position (wait until correct)", tf.getPosition())
            telemetry.update()
        }
        runtime.reset()

        val goldPosition = detectMineral(tf)

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
        hw.turn(-270f + when (goldPosition) {
            GoldPosition.LEFT -> 45f
            GoldPosition.CENTER -> 0f
            GoldPosition.RIGHT -> -30f
        })

        // Begin retracting the lifter
        hw.lifter.targetPosition = Hardware.LIFTER_AUTO_END_POSITION

        // Hit the gold mineral
        hw.drive(24.0) // far enough to always hit the mineral

        if (goldPosition == GoldPosition.RIGHT || goldPosition == GoldPosition.LEFT) {
            hw.turnFromStart(90f) // turn so back faces rover
        }
        hw.drive(-12.5) // change the amount as needed
        // Navigate toward depot (turn toward depot) and drive into wall
        hw.turn(85f)
        hw.drive(14.8 + when (goldPosition) {
            GoldPosition.LEFT -> 11.8
            GoldPosition.CENTER -> 38.2
            GoldPosition.RIGHT -> 47.25
        })

        when (startLocation) {
            AutonomousStartLocation.FACING_DEPOT -> {
                hw.turn(-115f)
                // Drive up to the depot
                hw.drive(35.0)
                // Release the claww
                hw.boxGate.position = 0.0 // toodles
                hw.rotateArmFromStartPosition(115f, power = 0.5, block = true)
                hw.rotateArmFromStartPosition(0f, block = false)
                // Drive toward the crater
                hw.turnFromStart(35f)
                hw.drive(-51.2)
            }
            AutonomousStartLocation.FACING_CRATER -> {
                hw.turn(45f)
                // Drive until depot and release the object
                hw.boxGate.position = 0.0 // toodles
                hw.rotateArmFromStartPosition(100f, power = 0.5, block = false)
                hw.drive(27.5)
                hw.rotateArmFromStartPosition(0f, block = false)
                hw.turn(15f)
                // Navigate back to crater
                hw.drive(-80.0)
            }
        }
        tf.deactivate()
    }
}
