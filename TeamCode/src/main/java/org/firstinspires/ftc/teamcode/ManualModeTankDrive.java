package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by nitro on 10/9/2017.
 */

@TeleOp(name = "Manual: tank drive")
public class ManualModeTankDrive extends LinearOpMode {
    private double limit(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    private void controlLimitedMotor(DcMotor motor, double bottomLimit, double topLimit, double controlAxis, double power) {
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

    @Override
    public void runOpMode() {
        Hardware hw = new Hardware(hardwareMap);

        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
            double turbo = 0.5;
            if (gamepad1.left_bumper) {
                turbo = 1;
            } else if (gamepad1.right_bumper) {
                turbo = 0.25;
            }

            hw.rightDriveMotor.setPower(gamepad1.right_stick_y * turbo);
            hw.leftDriveMotor.setPower(gamepad1.left_stick_y * turbo);

            hw.leftGrabberServo.setPosition(limit(gamepad2.left_trigger, Hardware.GRABBER_RELEASED, Hardware.GRABBER_GRABBED));
            hw.rightGrabberServo.setPosition(limit(gamepad2.left_trigger, Hardware.GRABBER_RELEASED, Hardware.GRABBER_GRABBED));

            controlLimitedMotor(
                    hw.lifterMotor,
                    0, Hardware.LIFTER_TOP_LIMIT,
                    -gamepad2.left_stick_y, 1);

            hw.relicHandServo.setPosition(limit(gamepad2.right_trigger, 0.5, 1.0));
            controlLimitedMotor(
                    hw.relicArmMotor,
                    0, Hardware.RELIC_ARM_TOP_LIMIT,
                    gamepad2.right_stick_y, 0.1);
            telemetry.addData("Left drive encoder value", hw.leftDriveMotor.getCurrentPosition());

            telemetry.update();
            idle();
        }
    }
}
