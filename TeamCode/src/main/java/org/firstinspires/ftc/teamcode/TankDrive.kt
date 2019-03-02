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
    private var boxGateClosed = true
    private var spinToggle = true
    private var leftTriggerPressed = false
    private var rightTriggerPressed = false

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
            hw.armExtender.power = 1.0
            hw.armExtender.targetPosition = Hardware.ARM_EXTENDER_BOTTOM_LIMIT
            hw.rotateArmFromStartPosition(0.0f, 0.2)
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
        val newPosition = (hw.leftArmRotator.currentPosition + (50 * armPower)).roundToInt()
        listOf(hw.leftArmRotator, hw.rightArmRotator).forEach {
            if (it.currentPosition < Hardware.ARM_ROTATION_MIDDLE_CHANGE) {
                it.mode = DcMotor.RunMode.RUN_TO_POSITION
                it.power = 0.7
                if (Math.abs(armPower) > 0.1) {
                    it.targetPosition = when {
                        // Let left bumper override limits on arm position
                        gamepad2.left_bumper -> newPosition
                        else -> limitValue(newPosition,
                                Hardware.ARM_ROTATION_BOTTOM_LIMIT, Hardware.ARM_ROTATION_UPPER_LIMIT)
                    }
                }
            } else {
                it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                it.power = armPower.toDouble()
            }
        }

        hw.armExtender.apply {
            power = 1.0
            var currentArmPosition = hw.armExtender.currentPosition
            if(gamepad2.left_bumper && gamepad2.dpad_up) targetPosition = currentArmPosition + 250
            else if(gamepad2.left_bumper && gamepad2.dpad_down) targetPosition = currentArmPosition - 250
            else if(gamepad2.dpad_up && currentPosition < Hardware.ARM_EXTENDER_UPPER_LIMIT) targetPosition = currentArmPosition + 250
            else if(gamepad2.dpad_down && currentPosition > Hardware.ARM_EXTENDER_BOTTOM_LIMIT) targetPosition = currentArmPosition - 250
        }

        telemetry.addData("Left arm encoder value", hw.leftArmRotator.currentPosition)
        if (hw.leftArmRotator.currentPosition < Hardware.ARM_ROTATION_MIDDLE_CHANGE) {
            telemetry.addData("Left arm target position", hw.leftArmRotator.targetPosition)
        } else {
            telemetry.addData("Left arm motor power", armPower)
        }
        telemetry.addData("Right arm encoder value", hw.rightArmRotator.currentPosition)
        if (hw.rightArmRotator.currentPosition < Hardware.ARM_ROTATION_MIDDLE_CHANGE) {
            telemetry.addData("Right arm target position", hw.rightArmRotator.targetPosition)
        } else {
            telemetry.addData("Right arm motor power", armPower)
        }
        telemetry.addData("Arm extender encoder value", hw.armExtender.currentPosition)
        telemetry.addData("Arm extender target position", hw.armExtender.targetPosition)
    }

    private fun runSweeper() {
        if (gamepad2.left_trigger > 0 && !leftTriggerPressed) {
            if(gamepad2.left_bumper){
                hw.boxSweeper.power = 1.0
                spinToggle = false
                leftTriggerPressed = true
            } else {
                hw.boxSweeper.power = when (spinToggle) {
                    true -> -1.0
                    else -> 0.0
                }
                spinToggle = !spinToggle
                leftTriggerPressed = true
            }
        } else if (gamepad2.left_trigger == 0.0f) leftTriggerPressed = false
    }

    private fun runBoxGate() {
        if (gamepad2.right_trigger > 0 && !rightTriggerPressed){
            hw.boxGate.position = when (boxGateClosed) {
                true -> 0.0
                false -> 1.0
            }
            boxGateClosed = !boxGateClosed
            rightTriggerPressed = true
        } else if (gamepad2.right_trigger == 0.0f) rightTriggerPressed = false
    }

    private fun runBox() {
        runSweeper()
        runBoxGate()
    }

    private fun runMacros() {
        when {
            gamepad2.a -> setBoxToCollect()
            gamepad2.x -> setBoxToCarry()
            gamepad2.y -> setBoxToDeposit()
        }
    }

    private fun setBoxToCollect() {
        setBoxToCarry()
        hw.boxGate.position = 1.0
        boxGateClosed = true
        hw.leftArmSupporter.position = Hardware.ARM_SUPPORTER_UP_POSITION
        hw.rightArmSupporter.position = Hardware.ARM_SUPPORTER_UP_POSITION
        spinToggle = true
        hw.boxSweeper.power = -1.0
        hw.rotateArmFromStartPosition(120f, 0.1, block = true)
        hw.rotateArmFromStartPosition(160f, 0.1, block = false)
    }

    private fun setBoxToDeposit() {
        setBoxToCarry()
        hw.armExtender.power = 1.0
        hw.armExtender.targetPosition = Hardware.ARM_EXTENDER_UPPER_LIMIT
        spinToggle = false
        hw.boxSweeper.power = 0.0
        hw.rotateArmFromStartPosition(53f, 0.3, block = false)
    }

    private fun setBoxToCarry() {
        hw.boxGate.position = 1.0
        boxGateClosed = true
        spinToggle = true
        hw.boxSweeper.power = 0.0
        hw.armExtender.targetPosition = Hardware.ARM_EXTENDER_UPPER_LIMIT/3
        hw.rotateArmFromStartPosition(80f, 1.0, block = true)
    }

    override fun runOpMode() {
        hw = Hardware(hardwareMap, this)

        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()
        runtime.reset()

        hw.stickServo.position = 0.0

        // run until the end of the match (driver presses STOP)
        val loopTime = ElapsedTime()

        while (opModeIsActive()) {
            loopTime.reset()
            runTankDrive()
            runLifter()
            runArm()
            runBox()
            runMacros()


            // Show the elapsed game time
            telemetry.addData("Status", "Run Time: $runtime")
            telemetry.addData("Loop time", loopTime.milliseconds().toString() + "ms")
            telemetry.addData("Encoder distance traveled (in.)",
                    hw.backRightDrive.currentPosition / Hardware.DRIVE_ENCODER_TICKS_PER_IN)
            telemetry.addData("IMU heading", hw.getHeading())
            telemetry.update()
        }
    }
}
