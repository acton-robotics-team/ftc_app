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

        hw.setLeftDrivePower(-gamepad1.left_stick_y * powerModifier)
        hw.setRightDrivePower(-gamepad1.right_stick_y * powerModifier)
    }

    /**
     * Left bumper = manual control, you're on your own
     */
    private fun runLifter(hw: Hardware) {
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

    private var armPreviouslyOnManualControl = false
    private var lastArmTargetPosition = 0
    private var wristPreviouslyOnManualControl = false
    private var lastWristTargetPosition = 0

    private fun runArm(hw: Hardware) {
        hw.arm.apply {
            if (Math.abs(gamepad2.right_stick_y) > 0.1) {
                armPreviouslyOnManualControl = true
                mode = DcMotor.RunMode.RUN_USING_ENCODER
                power = 0.3 * -gamepad2.right_stick_y
            } else {
                mode = DcMotor.RunMode.RUN_TO_POSITION
                power = 0.3
                if (armPreviouslyOnManualControl) {
                    armPreviouslyOnManualControl = false
                    lastArmTargetPosition = currentPosition
                }
                targetPosition = lastArmTargetPosition
            }
        }

        hw.wrist.apply {
            if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                wristPreviouslyOnManualControl = true
                mode = DcMotor.RunMode.RUN_USING_ENCODER
                power = 0.3 * -gamepad2.left_stick_y
            } else {
                mode = DcMotor.RunMode.RUN_TO_POSITION
                power = 0.3
                if (wristPreviouslyOnManualControl) {
                    wristPreviouslyOnManualControl = false
                    lastWristTargetPosition = currentPosition
                }
                targetPosition = lastWristTargetPosition
            }
        }
        
        hw.armExtender.apply {
            mode = DcMotor.RunMode.RUN_TO_POSITION
            power = 0.5
            targetPosition = when {
                gamepad2.dpad_up -> Math.min(currentPosition + 200, Hardware.ARM_EXTENDED)
                gamepad2.dpad_down -> Math.max(currentPosition - 200, Hardware.ARM_RETRACTED)
                else -> targetPosition
            }
        }

        telemetry.addData("Arm encoder value", hw.arm.currentPosition)
        telemetry.addData("Arm target position", hw.arm.targetPosition)
        telemetry.addData("Wrist encoder position", hw.wrist.currentPosition)
        telemetry.addData("Wrist encoder target position", hw.wrist.targetPosition)
        telemetry.addData("Arm extender encoder value", hw.armExtender.currentPosition)
        telemetry.addData("Arm extender target position", hw.armExtender.targetPosition)
    }

    private fun runArmGrabber(hw: Hardware) {
        hw.grabber.position = 1 - gamepad2.left_trigger.toDouble()
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
