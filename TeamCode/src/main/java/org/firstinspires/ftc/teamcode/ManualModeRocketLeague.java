package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by nitro on 10/9/2017.
 */

@TeleOp(name = "Manual: rocket league")
public class ManualModeRocketLeague extends LinearOpMode {
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

    @Override
    public void runOpMode() {
        Hardware hw = new Hardware(hardwareMap);

        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                // Turning to the left
                hw.leftDriveMotor.setPower(-1);
                hw.rightDriveMotor.setPower(1);
            } else if (gamepad1.right_bumper) {
                // Turning to the right
                hw.leftDriveMotor.setPower(1);
                hw.rightDriveMotor.setPower(-1);
            } else {
                float frontPower = gamepad1.right_trigger - gamepad1.left_trigger;

                if (gamepad1.left_stick_x > 0) {
                    // Turning to the right
                    hw.rightDriveMotor.setPower(frontPower - gamepad1.left_stick_x * 0.5);
                    hw.leftDriveMotor.setPower(frontPower);
                } else {
                    // Turning to the left (left_stick_x is now negative)
                    hw.leftDriveMotor.setPower(frontPower + gamepad1.left_stick_x * 0.5);
                    hw.rightDriveMotor.setPower(frontPower);
                }
            }


            hw.leftGrabberServo.setPosition(limit(gamepad2.left_trigger, 0.5, 1.0));
            hw.rightGrabberServo.setPosition(limit(gamepad2.left_trigger, 0.5, 1.0));

            controlLimitedMotor(
                    hw.lifterMotor,
                    0, 4 * Hardware.TETRIX_TICKS_PER_REVOLUTION,
                    -gamepad2.left_stick_y, 1);

            hw.relicHandServo.setPosition(limit(gamepad2.right_trigger, 0.5, 1.0));
            controlLimitedMotor(
                    hw.relicArmMotor,
                    0, 0.5 * Hardware.TETRIX_TICKS_PER_REVOLUTION,
                    gamepad2.right_stick_y, 0.1);
            idle();
        }
    }
}
