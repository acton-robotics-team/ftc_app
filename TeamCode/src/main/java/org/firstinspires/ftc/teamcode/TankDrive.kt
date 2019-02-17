package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.roundToInt
import java.lang.IndexOutOfBoundsException

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
    var boxPosition = 1
    var spinToggle = true
    var leftTriggerPressed = false
    var rightTriggerPressed = false
    var rightBumperPressed = false

    private fun runTankDrive() {
        val powerModifier = if (gamepad1.a) Hardware.DRIVE_SLOW else Hardware.DRIVE_FAST

        hw.setLeftDrivePower(-gamepad1.left_stick_y * powerModifier)
        hw.setRightDrivePower(-gamepad1.right_stick_y * powerModifier)
    }

    private fun limitValue(value: Int, min: Int, max: Int): Int {
        return Math.min(Math.max(value, min), max)
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
            it.mode = DcMotor.RunMode.RUN_TO_POSITION
            it.power = 0.7
            if (Math.abs(armPower) > 0.1) {
                val newPosition = (it.currentPosition + (50 * armPower)).roundToInt()
                it.targetPosition = when {
                    // Let left bumper override limits on arm position
                    gamepad2.left_bumper -> newPosition
                    else -> limitValue(newPosition,
                            Hardware.ARM_ROTATION_BOTTOM_LIMIT, Hardware.ARM_ROTATION_UPPER_LIMIT)
                }

            }
        }

        hw.armExtender.power = when {
            gamepad2.dpad_up -> -0.5
            gamepad2.dpad_down -> 0.5
            else -> 0.0
        }

        telemetry.addData("Left arm encoder value", hw.leftArmRotator.currentPosition)
        telemetry.addData("Left arm target position", hw.leftArmRotator.targetPosition)
        telemetry.addData("Right arm encoder value", hw.rightArmRotator.currentPosition)
        telemetry.addData("Right arm target position", hw.rightArmRotator.targetPosition)
        telemetry.addData("Arm extender encoder value", hw.armExtender.currentPosition)
        telemetry.addData("Arm extender target position", hw.armExtender.targetPosition)
    }

    private fun runSweeper(){
        if (boxPosition == 2) hw.boxSweeper.power = 0.0
        else if (gamepad2.left_trigger > 0 && !leftTriggerPressed) {
            hw.boxSweeper.power = when (spinToggle) {
                true -> 0.7
                else -> 0.0
            }
            spinToggle = !spinToggle
            leftTriggerPressed = true
        }
        else if (gamepad2.left_trigger == 0.0f) leftTriggerPressed = false
    }

    private fun runBoxHinge(){
        if (gamepad2.right_bumper && !rightBumperPressed) {
            if(boxPosition < 2) boxPosition++
            rightBumperPressed = true
        }
        else if (!gamepad2.right_bumper) rightBumperPressed = false
        if (gamepad2.right_trigger > 0 && !rightTriggerPressed) {
            if(boxPosition > 0) boxPosition--
            rightTriggerPressed = true
        }
        else if (gamepad2.right_trigger == 0.0f) rightTriggerPressed = false
        hw.boxHingeServo.position = when (boxPosition) {
            0 -> 0.0
            1 -> 0.45
            2 -> 1.0
            else -> throw IndexOutOfBoundsException("box position out of bounds")
        }
    }

    private fun runBox() {
        runSweeper()
        runBoxHinge()

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

            // Show the elapsed game time
            telemetry.addData("Status", "Run Time: $runtime")
            telemetry.addData("Loop time", loopTime.milliseconds().toString() + "ms")
            telemetry.addData("Encoder distance traveled (in.)",
                    hw.backRightDrive.currentPosition / Hardware.DRIVE_ENCODER_TICKS_PER_IN)
            telemetry.addData("IMU heading", hw.getHeading())
            if (gamepad1.a) {
                hw.backRightDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                hw.backRightDrive.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }
            telemetry.update()
        }
    }
}
