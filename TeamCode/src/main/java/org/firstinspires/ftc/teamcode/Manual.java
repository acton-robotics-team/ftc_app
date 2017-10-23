package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by nitro on 10/9/2017.
 */

@TeleOp(name = "Manual program")
public class Manual extends LinearOpMode {
    private double limit(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }
    private int limit(int value, int min, int max) {
        return Math.min(Math.max(value, min), max);
    }

    @Override
    public void runOpMode() {
        Hardware hw = new Hardware(hardwareMap);

        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
            hw.rightDriveMotor.setPower(gamepad1.right_stick_y * 0.5);
            hw.leftDriveMotor.setPower(gamepad1.left_stick_y * 0.5);

            hw.leftGrabberServo.setPosition(limit(gamepad2.left_trigger, 0.5, 1.0));
            hw.rightGrabberServo.setPosition(limit(gamepad2.right_trigger, 0.5, 1.0));

            int offset;
            int position = hw.lifterMotor.getCurrentPosition();
            telemetry.addData("Encoder Position", position);
            hw.lifterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (gamepad2.right_stick_y > 0.1 && position < 0) {
                // Actually pushing down -- positive offset = move DOWN
                hw.lifterMotor.setPower(1);
            } else if (gamepad2.right_stick_y < -0.1 && position > -4500) {
                // Actually pushing up -- negative offset = move UP
                hw.lifterMotor.setPower(-1);
            }
            else{
                hw.lifterMotor.setPower(0);
            }
            /*hw.lifterMotor.setTargetPosition(limit(
                    hw.lifterMotor.getCurrentPosition() + offset,
                    0, // Do not go below the initial position.
                    (int) (4.5 * Hardware.TETRIX_TICKS_PER_REVOLUTION))); // Do not go more than 4.5 revolutions*/
            idle();
        }
    }
}