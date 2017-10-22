package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by nitro on 10/9/2017.
 */

@TeleOp(name = "Manual program")
public class Manual extends LinearOpMode {
    @Override
    public void runOpMode() {
        Hardware hw = new Hardware(hardwareMap);

        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
            hw.rightDriveMotor.setPower(gamepad1.right_stick_y);
            hw.leftDriveMotor.setPower(gamepad1.left_stick_y);

            hw.lifterMotor.setTargetPosition(
                    hw.lifterMotor.getCurrentPosition() + (int)Math.floor(gamepad2.right_stick_y)
            );

            hw.leftGrabberServo.setPosition(gamepad2.left_trigger);
            hw.rightGrabberServo.setPosition(gamepad2.right_trigger);

            idle();
        }
    }
}
