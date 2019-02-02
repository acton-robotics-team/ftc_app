package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
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

@TeleOp(name = "Tank Drive", group = "Linear Opmode")
class TankDrive : LinearOpMode() {
    private val runtime = ElapsedTime()
    private lateinit var hw: Hardware

    private fun runTankDrive() {
        val powerModifier = if (gamepad1.a) Hardware.DRIVE_SLOW else Hardware.DRIVE_FAST

        hw.setLeftDrivePower(-gamepad1.left_stick_y * powerModifier)
        hw.setRightDrivePower(-gamepad1.right_stick_y * powerModifier)
    }

    /**
     * Left bumper = manual control, you're on your own
     */
    private fun runLifter() {
        if (gamepad1.left_bumper) {
            hw.lifter.mode = DcMotor.RunMode.RUN_TO_POSITION
            hw.lifter.power = 1.0
            hw.lifter.targetPosition = Hardware.LIFTER_TOP_POSITION
        } else {
            hw.lifter.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            when {
                gamepad1.dpad_up -> hw.lifter.power = 0.5
                gamepad1.dpad_down -> hw.lifter.power = -0.5
                else -> hw.lifter.power = 0.0
            }
        }
        telemetry.addData("Lifter encoder value", hw.lifter.currentPosition)
    }

    private fun runArm() {
        val armPower = -gamepad2.right_stick_y
        listOf(hw.leftArmRotator, hw.rightArmRotator).forEach {
            it.apply {
                mode = DcMotor.RunMode.RUN_TO_POSITION
                power = 0.5
                if (armPower > 0.1) {
                    targetPosition = Math.min(500, currentPosition + (50 * Math.abs(armPower)).roundToInt())
                } else if (armPower < -0.1) {
                    targetPosition = Math.max(0, currentPosition - (50 * Math.abs(armPower)).roundToInt())
                }
            }
        }
        hw.armExtender.power = when {
            gamepad2.dpad_up -> 0.5
            gamepad2.dpad_down -> -0.5
            else -> 0.0
        }

        telemetry.addData("Left arm encoder value", hw.leftArmRotator.currentPosition)
        telemetry.addData("Left arm target position", hw.leftArmRotator.targetPosition)
        telemetry.addData("Right arm encoder value", hw.rightArmRotator.currentPosition)
        telemetry.addData("Right arm target position", hw.rightArmRotator.targetPosition)
        telemetry.addData("Arm extender encoder value", hw.armExtender.currentPosition)
        telemetry.addData("Arm extender target position", hw.armExtender.targetPosition)
    }

    private fun runBox() {
        hw.boxSweeper.power = when (hw.leftBoxHingeServo.position) {
            0.0 -> 0.7
            else -> 0.0
        }
        hw.setHingeServoPosition((gamepad2.right_trigger).toDouble())
    }

    private fun runMacros() {
        // Gamepad 2, A btn
//        if (gamepad2.a) {
//            leftArmRotator.setTargetPosition(Hardware.ARM_GRABBING_POSITION)
//            rightArmRotator.setTargetPosition(Hardware.ARM_GRABBING_POSITION)
//        }
//        // Gamepad 2, X btn
//        else if (gamepad2.x) {
//            leftArmRotator.setTargetPosition(Hardware.ARM_SCORING_POSITION)
//            rightArmRotator.setTargetPosition(Hardware.ARM_GRABBING_POSITION)
//        }
    }

    override fun runOpMode() {
        hw = Hardware(hardwareMap, this)

        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()
        runtime.reset()

        // run until the end of the match (driver presses STOP)
        val loopTime = ElapsedTime()
        while (opModeIsActive()) {
            loopTime.reset()
            runTankDrive()
            runLifter()
            runArm()
            runBox()
//            runMacros()

            // Show the elapsed game time
            telemetry.addData("Status", "Run Time: $runtime")
            telemetry.addData("Loop time", loopTime.milliseconds().toString() + "ms")
            telemetry.update()
        }
    }
}
