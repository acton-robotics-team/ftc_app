package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by nitro on 10/9/2017.
 */

@TeleOp(name = "Manual program")
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.servo.get("servo");
        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
            servo.setPosition(gamepad2.left_trigger);
            idle();
        }
    }
}
