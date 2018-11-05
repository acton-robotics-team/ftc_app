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

@TeleOp(name = "Drive Measurement", group = "Linear Opmode")
class DriveMeasurement : LinearOpMode() {
    private val runtime = ElapsedTime()


    override fun runOpMode() {
        val hw = Hardware(hardwareMap)

        hw.leftDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        hw.rightDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        hw.rightDrive.mode = DcMotor.RunMode.RUN_TO_POSITION
        hw.leftDrive.mode = DcMotor.RunMode.RUN_TO_POSITION

        telemetry.addData("Status", "Initialized")
        telemetry.update()

        // Wait for the game to start (driver presses PLAY)
        waitForStart()
        runtime.reset()

        hw.rightDrive.targetPosition = (30 * Hardware.DRIVE_ENCODER_TICKS_PER_CM).roundToInt()
        hw.leftDrive.targetPosition = (30 * Hardware.DRIVE_ENCODER_TICKS_PER_CM).roundToInt()
        hw.rightDrive.power = Hardware.DRIVE_SLOWEST
        hw.leftDrive.power = Hardware.DRIVE_SLOWEST

        while (opModeIsActive() && (hw.leftDrive.isBusy || hw.rightDrive.isBusy)) {
            telemetry.addData("Left", hw.leftDrive.currentPosition)
            telemetry.addData("Right", hw.leftDrive.currentPosition)
            telemetry.update()
            idle()
        }

        hw.rightDrive.power = 0.0
        hw.leftDrive.power = 0.0

        telemetry.addLine("Ran for ${runtime.milliseconds()} ms")
        telemetry.addData("Left", hw.leftDrive.currentPosition)
        telemetry.addData("Right", hw.leftDrive.currentPosition)
        telemetry.update()
    }
}
