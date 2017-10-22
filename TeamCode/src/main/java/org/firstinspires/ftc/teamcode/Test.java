package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by nitro on 10/9/2017.
 */

@TeleOp(name = "Test")
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo leftGrabberServo = hardwareMap.servo.get("left_grabber_servo");
        Servo rightGrabberServo = hardwareMap.servo.get("right_grabber_servo");
        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
            leftGrabberServo.setPosition(gamepad2.left_trigger);
            rightGrabberServo.setDirection(Servo.Direction.REVERSE);
            rightGrabberServo.setPosition(gamepad2.right_trigger);
            idle();
        }
    }
}
