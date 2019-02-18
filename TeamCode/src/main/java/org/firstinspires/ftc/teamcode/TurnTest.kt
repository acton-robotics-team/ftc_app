package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "Turn Test", group = "Linear Opmode")
class TurnTest : LinearOpMode() {
    private lateinit var hw: Hardware
    var turnVal = 0.0f
    var aPressed = false

    override fun runOpMode() {
        hw = Hardware(hardwareMap, this)
        waitForStart()
        while(opModeIsActive()){
            turnVal += when{
                gamepad1.left_stick_y > 0 -> 1
                gamepad1.left_stick_y < 0 -> -1
                else -> 0
            }

            if (gamepad1.a && !aPressed) {
                hw.turn(turnVal)
                aPressed = true
                turnVal = 0.0f;
            }
            else if (gamepad1.a) aPressed = false

            telemetry.addData("Turn Ammount", turnVal)
            telemetry.update()
        }
    }
}