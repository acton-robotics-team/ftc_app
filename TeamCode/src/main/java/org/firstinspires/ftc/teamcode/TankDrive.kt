package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
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

@TeleOp(name = "Basic: Linear OpMode", group = "Linear Opmode")
class TankDrive : LinearOpMode() {
    private val runtime = ElapsedTime()

    private fun runTankDrive(hw: Hardware) {
        if (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_y) > 0.1) {
            // First gamepad control overrides right's
            val powerModifier = if (gamepad1.a) Hardware.SLOW_SPEED else Hardware.FULL_SPEED

            hw.leftMotor.power = gamepad1.left_stick_y * powerModifier
            hw.rightMotor.power = gamepad1.right_stick_y * powerModifier
        } else {
            // Second gamepad is always slow
            hw.leftMotor.power = gamepad2.left_stick_y * Hardware.SLOW_SPEED
            hw.rightMotor.power = gamepad2.right_stick_y * Hardware.SLOW_SPEED
        }
    }

    //something
    private fun runLifter(hw: Hardware) {
        if (gamepad1.dpad_up) {
            hw.lifter.power = 1.0
            hw.lifter.targetPosition = Hardware.LIFTER_TOP_POSITION
        } else if (gamepad1.dpad_down) {
            hw.lifter.power = -1.0
            hw.lifter.targetPosition = Hardware.LIFTER_BOTTOM_POSITION
        }
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

            // Show the elapsed game time
            telemetry.addData("Status", "Run Time: " + runtime.toString())
            telemetry.addData("Lifter encoder value", hw.lifter.currentPosition)
            telemetry.update()
        }
    }
}
