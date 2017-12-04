package com.github.actonroboticsteam.ftcapp


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.pattonvillerobotics.commoncode.robotclasses.gamepad.GamepadData
import org.pattonvillerobotics.commoncode.robotclasses.gamepad.ListenableButton
import org.pattonvillerobotics.commoncode.robotclasses.gamepad.ListenableGamepad

/**
 * Created by kevinliu who writes tehe mstdisgusting code ew
 * on 10/9/2017.
 * - kuneel sharpedo
 *
 * Controls:
 *
 * GAMEPAD 1
 * - Left bumper: turbo (=1)
 * - Right bumper: slow (=0.125)
 * - Right/left analog sticks: drive motors
 * GAMEPAD 2
 * - Left trigger: grabber servos
 * - Right trigger: relic hand servo
 * - Up/down dpad: extend/retract slide (slide lifter servo)
 * - Left/right dpad: lift/lower slide (slide extender servo)
 * - Y: toggle slide gate servo
 * - Left analog stick: lifter motor
 * - NOT A && Right analog stick: relic arm motor
 * - A && right analog stick: relic elbow servo
 * - B: relic hand servo
 */
@TeleOp(name = "Manual: tank drive")
class ManualModeTankDrive : LinearOpMode() {
    private val runtime = ElapsedTime()

    private fun limit(value: Double, min: Double, max: Double): Double {
        return Math.min(Math.max(value, min), max)
    }

    private fun controlLimitedMotor(
            motor: DcMotor, bottomLimit: Double, topLimit: Double,
            controlAxis: Double, power: Double) {
        val position = motor.currentPosition
        telemetry.addLine("LIMITED MOTOR")
        telemetry.addData("Control axis", controlAxis)
        telemetry.addData("Position", position)
        telemetry.addData("Bottom limit", bottomLimit)
        telemetry.addData("Top limit", topLimit)
        if (controlAxis > 0.1 && position < topLimit) {
            telemetry.addLine("LIMIT: moving up")
            motor.power = power
        } else if (controlAxis < -0.1 && position > bottomLimit) {
            telemetry.addLine("LIMIT: moving down")
            motor.power = -power
        } else {
            telemetry.addLine("LIMIT: no control input")
            motor.power = 0.0
        }
    }

    private fun fineControlServo(
            servo: Servo, bottomLimit: Double, topLimit: Double,
            isUp: Boolean, isDown: Boolean) {
        val delta = when {
            isUp -> 0.05
            isDown -> -0.05
            else -> 0.0
        }
        val position = if (servo.position.isNaN()) {
            // freaking servo api returns NaN when no value known
            bottomLimit
        } else {
            servo.position
        }
        telemetry.addLine("CONTROL SERVO")
        telemetry.addData("Position", position)
        telemetry.addData("Bottom limit", bottomLimit)
        telemetry.addData("Top limit", topLimit)
        telemetry.addData("Is up?", isUp)
        telemetry.addData("Is down?", isDown)
        telemetry.addData("Adding delta", delta)
        servo.position = limit(
                position + delta, bottomLimit, topLimit)
    }

    override fun runOpMode() {
        val hw = RobotConfig(hardwareMap)
        val gamepad2Listener = ListenableGamepad()
        gamepad2Listener.addButtonListener(
                GamepadData.Button.Y, ListenableButton.ButtonState.JUST_PRESSED) {
            hw.slideGateServo.position =
                    if (hw.slideGateServo.position == RobotConfig.SLIDE_GATE_CLOSED) {
                RobotConfig.SLIDE_GATE_OPEN
            } else {
                RobotConfig.SLIDE_GATE_CLOSED
            }
        }
        gamepad2Listener.addButtonListener(
                GamepadData.Button.B, ListenableButton.ButtonState.JUST_PRESSED) {
            hw.relicHandServo.position =
                    if (hw.relicHandServo.position == RobotConfig.RELIC_HAND_CLOSED) {
                RobotConfig.RELIC_HAND_OPEN
            } else {
                RobotConfig.RELIC_HAND_CLOSED
            }
        }

        // wait for the start button to be pressed.
        waitForStart()
        runtime.reset()

        while (opModeIsActive()) {
            try {
                telemetry.addData("Runtime", runtime)

                // Gamepad 1
                val turbo = when {
                    gamepad1.left_bumper -> 1.0
                    gamepad1.right_bumper -> 0.125
                    else -> 0.25
                }

                hw.rightDriveMotor.power = gamepad1.right_stick_y * turbo
                hw.leftDriveMotor.power = gamepad1.left_stick_y * turbo

                // Gamepad 2
                hw.leftGrabberServo.position = limit(gamepad2.left_trigger.toDouble(), RobotConfig.GRABBER_RELEASED, RobotConfig.GRABBER_GRABBED)
                hw.rightGrabberServo.position = limit(gamepad2.left_trigger.toDouble(), RobotConfig.GRABBER_RELEASED, RobotConfig.GRABBER_GRABBED)

                fineControlServo(hw.slideLifterServo,
                        0.0, 1.0,
                        gamepad2.dpad_left, gamepad2.dpad_right)
                fineControlServo(hw.slideExtenderServo,
                        0.0, 1.0,
                        gamepad2.dpad_up, gamepad2.dpad_down)

                gamepad2Listener.update(gamepad2)

                telemetry.addData("Slide gate servo position", hw.slideGateServo.position)

                controlLimitedMotor(
                        hw.lifterMotor,
                        0.0, RobotConfig.LIFTER_TOP_LIMIT.toDouble(),
                        (-gamepad2.left_stick_y).toDouble(), 1.0)

                // right stick: motor for arm
                // a+right stick: servo for elbow
                // b: relic hand
                telemetry.addData("Relic hand position", hw.relicHandServo.position)

                if (gamepad2.a) {
                    hw.relicArmMotor.power = 0.0
                    fineControlServo(hw.relicElbowServo,
                            0.0, 1.0,
                            gamepad2.right_stick_y > 0, gamepad2.right_stick_y < 0)
                } else {
                    controlLimitedMotor(
                            hw.relicArmMotor,
                            0.0, RobotConfig.RELIC_ARM_TOP_LIMIT,
                            gamepad2.right_stick_y.toDouble(), 0.3)
                }
                telemetry.addData("Left drive encoder value",
                        hw.leftDriveMotor.currentPosition)
            } catch (e: Exception) {
                // Global exception handler to get backtrace
                telemetry.addLine("EXCEPTION:")
                telemetry.addLine(e.stackTrace.contentToString())
            } finally {
                telemetry.update()
                idle()
            }
        }
    }
}
