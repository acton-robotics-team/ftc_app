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

    private fun runTankDrive(hw: Hardware) {
        val powerModifier = if (gamepad1.a) Hardware.DRIVE_SLOW else Hardware.DRIVE_FAST

        hw.leftBottomDrive.power = -gamepad1.left_stick_y * powerModifier
        hw.rightBottomDrive.power = -gamepad1.right_stick_y * powerModifier
    }

    /**
     * Left bumper = manual control, you're on your own
     */
    private fun runLifter(hw: Hardware) {
        if (gamepad1.left_bumper) {
            hw.lifter.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            hw.lifter.power = when {
                gamepad1.dpad_up -> 0.5
                gamepad1.dpad_down -> -0.5
                else -> 0.0
            }
        } else {
            hw.lifter.mode = DcMotor.RunMode.RUN_TO_POSITION
            when {
                gamepad1.dpad_up -> {
                    hw.lifter.power = 1.0
                    hw.lifter.targetPosition = Hardware.LIFTER_TOP_POSITION
                }
                gamepad1.dpad_down -> {
                    hw.lifter.power = -1.0
                    hw.lifter.targetPosition = Hardware.LIFTER_BOTTOM_POSITION
                }
            }
        }
        telemetry.addData("Lifter encoder value", hw.lifter.currentPosition)
    }

    private fun runArm(hw: Hardware) {
        hw.arm.power = -gamepad2.left_stick_y.toDouble()
        hw.wrist.power = -gamepad2.right_stick_y.toDouble()
        hw.armExtender.power = when {
            gamepad2.dpad_up -> 1.0
            gamepad2.dpad_down -> -1.0
            else -> 0.0
        }
        telemetry.addData("Extender encoder value", hw.arm.currentPosition)
    }

    private fun runArmGrabber(hw: Hardware) {
        hw.leftGrabber.position = gamepad1.left_trigger.toDouble()
        hw.rightGrabber.position = gamepad1.right_trigger.toDouble()
    }

    override fun runOpMode() {
        telemetry.addData("Status", "Initialized")
        telemetry.update()

        val hw = Hardware(hardwareMap)

        // Wait for the game to start (driver presses PLAY)
        waitForStart()
        runtime.reset()

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            runTankDrive(hw)
            runLifter(hw)
            runArm(hw)
            runArmGrabber(hw)

            // Show the elapsed game time
            telemetry.addData("Status", "Run Time: " + runtime.toString())
            telemetry.update()
        }
    }
}
