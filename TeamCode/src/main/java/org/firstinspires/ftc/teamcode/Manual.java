package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by nitro on 10/9/2017.
 */

@TeleOp(name = "Manual program")
public class Manual extends LinearOpMode {
    private double limit(double value, double min, double max) {
        return Math.min(value + min, max);
    }

    @Override
    public void runOpMode() {
        Hardware hw = new Hardware(hardwareMap);

        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
            hw.rightDriveMotor.setPower(gamepad1.right_stick_y);
            hw.leftDriveMotor.setPower(gamepad1.left_stick_y);

            hw.leftGrabberServo.setPosition(limit(gamepad2.left_trigger, 0.2, 1.0));
            hw.rightGrabberServo.setPosition(limit(gamepad2.right_trigger, 0.2, 1.0));

            idle();
        }
    }
}
