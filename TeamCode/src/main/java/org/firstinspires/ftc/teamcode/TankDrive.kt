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

        hw.leftDrive.power = -gamepad1.left_stick_y * powerModifier
        hw.rightDrive.power = -gamepad1.right_stick_y * powerModifier
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
                    hw.lifter.power = 1.0
                    hw.lifter.targetPosition = Hardware.LIFTER_BOTTOM_POSITION
                }
            }
        }
        telemetry.addData("Lifter encoder value", hw.lifter.currentPosition)
    }

    private var armTarget = Hardware.ARM_DOWN

    private fun runArm(hw: Hardware) {
        hw.arm.apply {
            if (gamepad2.right_bumper) {
                mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                power = 0.5 * gamepad2.right_stick_y
            } else if (gamepad2.x) {
                mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                power = 0.0
            } else if (gamepad2.y || gamepad2.a) {
                mode = DcMotor.RunMode.RUN_TO_POSITION
                power = 0.5
                armTarget = when {
                    gamepad2.y -> Hardware.ARM_HALF_UP
                    gamepad2.a -> Hardware.ARM_UP
                    else -> armTarget
                }
                targetPosition = armTarget
            }
        }

        // Just direct set power because we don't have an encoder on it
        hw.wrist.power = 0.4 * -gamepad2.left_stick_y.toDouble()

        hw.armExtender.apply {
            if (gamepad2.left_bumper) {
                mode = DcMotor.RunMode.RUN_USING_ENCODER
                power = when {
                    gamepad2.dpad_up -> 0.5
                    gamepad2.dpad_down -> -0.5
                    else -> 0.0
                }
            } else {
                when {
                    gamepad2.dpad_up -> {
                        mode = DcMotor.RunMode.RUN_TO_POSITION
                        power = 0.5
                        targetPosition = Hardware.ARM_EXTENDED
                    }
                    gamepad2.dpad_down -> {
                        mode = DcMotor.RunMode.RUN_TO_POSITION
                        power = 0.5
                        targetPosition = Hardware.ARM_RETRACTED
                    }
                }
            }
        }

        telemetry.addData("Arm encoder value", hw.arm.currentPosition)
        telemetry.addData("Arm target position", hw.arm.targetPosition)
        telemetry.addData("Wrist encoder value", hw.wrist.currentPosition)
        telemetry.addData("Arm extender encoder value", hw.armExtender.currentPosition)
        telemetry.addData("Arm extender target position", hw.armExtender.targetPosition)
    }

    private fun runArmGrabber(hw: Hardware) {
        hw.leftGrabber.position = 1 - gamepad1.left_trigger.toDouble()
        hw.rightGrabber.position = 1 - gamepad1.right_trigger.toDouble()
    }

    override fun runOpMode() {
        telemetry.addData("Status", "Initialized")
        telemetry.update()

        val hw = Hardware(hardwareMap)

        // Wait for the game to start (driver presses PLAY)
        waitForStart()
        runtime.reset()

        // run until the end of the match (driver presses STOP)
        val loopTime = ElapsedTime()
        while (opModeIsActive()) {
            loopTime.reset()
            runTankDrive(hw)
            runLifter(hw)
            runArm(hw)
            runArmGrabber(hw)

            // Show the elapsed game time
            telemetry.addData("Status", "Run Time: " + runtime.toString())
            telemetry.addData("Loop time", loopTime.milliseconds().toString() + "ms")
            telemetry.update()
        }
    }
}
