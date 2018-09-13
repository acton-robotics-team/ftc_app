package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import kotlin.math.abs

/**
 * Created by kevin on 1/21/18.
 */

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Basic: Iterative OpMode", group = "Iterative Opmode")
class DemoOpMode : OpMode() {
    // Declare OpMode members.
    private val runtime = ElapsedTime()
    private var hardware: Hardware? = null

    /*
     * Code to run ONCE when the driver hits INIT
     */
    override fun init() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    override fun init_loop() {}

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    override fun start() {
        hardware = Hardware(hardwareMap)

        runtime.reset()
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    override fun loop() {
        if (abs(gamepad1.right_stick_x) > 0.1) {
            // turny mode
            listOf(hardware!!.topDrive, hardware!!.rightDrive, hardware!!.bottomDrive, hardware!!.leftDrive).forEach {
                it.power = -gamepad1.right_stick_x.toDouble()
            }
        } else {
            // movey mode
            hardware!!.leftDrive.power = gamepad1.left_stick_y.toDouble()
            hardware!!.rightDrive.power = -gamepad1.left_stick_y.toDouble()
            hardware!!.topDrive.power = -gamepad1.left_stick_x.toDouble()
            hardware!!.bottomDrive.power = gamepad1.left_stick_x.toDouble()
            hardware = null
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    override fun stop() {}

}
