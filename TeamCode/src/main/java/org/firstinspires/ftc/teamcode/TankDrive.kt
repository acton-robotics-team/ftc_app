package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
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

@TeleOp(name = "Tank Drive", group = "Linear Opmode")
class TankDrive : LinearOpMode() {
    private val runtime = ElapsedTime()
    private lateinit var hw: Hardware
    private lateinit var armRotator1: PositionHoldingMotor
    private lateinit var armRotator2: PositionHoldingMotor

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
        armRotator1.processInput(-gamepad2.right_stick_y)
        armRotator2.processInput(-gamepad2.right_stick_y)
        hw.armExtender.apply {
            mode = DcMotor.RunMode.RUN_TO_POSITION
            power = 0.3
            targetPosition = when {
                gamepad2.dpad_up -> Math.min(currentPosition + 100, Hardware.ARM_EXTENDED)
                gamepad2.dpad_down -> Math.max(currentPosition - 100, Hardware.ARM_RETRACTED)
                else -> targetPosition
            }
        }

        telemetry.addData("Arm encoder value", hw.armRotator1.currentPosition)
        telemetry.addData("Arm target position", hw.armRotator1.targetPosition)
        telemetry.addData("Arm extender encoder value", hw.armExtender.currentPosition)
        telemetry.addData("Arm extender target position", hw.armExtender.targetPosition)
    }

    private fun runArmGrabber() {
        hw.grabber.position = 1 - gamepad2.left_trigger.toDouble()
    }

    private fun runMacros() {
        // Gamepad 2, A btn
        if (gamepad2.a) {
            armRotator1.setTargetPosition(Hardware.ARM_GRABBING_POSITION)
            armRotator2.setTargetPosition(Hardware.ARM_GRABBING_POSITION)
        }
        // Gamepad 2, X btn
        else if (gamepad2.x) {
            armRotator1.setTargetPosition(Hardware.ARM_SCORING_POSITION)
            armRotator2.setTargetPosition(Hardware.ARM_GRABBING_POSITION)
        }
    }

    override fun runOpMode() {
        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()
        runtime.reset()

        // Move back to initial position after autonomous mode has extended it
        hw = Hardware(hardwareMap)
        armRotator1 = PositionHoldingMotor(hw.armRotator1, 0.3)
        armRotator2 = PositionHoldingMotor(hw.armRotator2, 0.3)

        // Wait for the game to start (driver presses PLAY)
        hw.armExtender.targetPosition = 0 // wtf

        // run until the end of the match (driver presses STOP)
        val loopTime = ElapsedTime()
        while (opModeIsActive()) {
            loopTime.reset()
            runTankDrive()
            runLifter()
            runArm()
            runArmGrabber()
            runMacros()

            // Show the elapsed game time
            telemetry.addData("Status", "Run Time: " + runtime.toString())
            telemetry.addData("Loop time", loopTime.milliseconds().toString() + "ms")
            telemetry.update()
        }
    }
}
