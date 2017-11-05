package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by nitro on 10/9/2017.
 */

@TeleOp(name = "ManualMode program")
public class ManualMode extends LinearOpMode {
    private double limit(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }
    public void controlLimitedMotor(DcMotor motor, double bottomLimit, double topLimit, double controlAxis, double power) {
        int position = motor.getCurrentPosition();
        telemetry.addData(
                "LIMIT motor " + motor.getDeviceName() + " CTRL " + controlAxis +
                        " POS " + position + " BOTTOM " + bottomLimit + " TOP " + topLimit, "");
        if (controlAxis > 0.1 && position < topLimit) {
            motor.setPower(power);
        } else if (controlAxis < -0.1 && position > bottomLimit) {
            motor.setPower(-power);
        } else {
            motor.setPower(0);
        }
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
            hw.rightGrabberServo.setPosition(limit(gamepad2.left_trigger, 0.5, 1.0));

            controlLimitedMotor(
                    hw.lifterMotor,
                    0, 4.5 * Hardware.TETRIX_TICKS_PER_REVOLUTION,
                    gamepad2.left_stick_y, 1);

            hw.relicHandServo.setPosition(limit(gamepad2.right_trigger, 0.5, 1.0));
            controlLimitedMotor(
                    hw.relicArmMotor,
                    0, 0.5 * Hardware.TETRIX_TICKS_PER_REVOLUTION,
                    gamepad2.right_stick_y, 0.1);
            idle();
        }
    }
}
