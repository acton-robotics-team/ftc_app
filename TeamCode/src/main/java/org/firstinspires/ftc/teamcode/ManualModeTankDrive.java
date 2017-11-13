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
    public void controlLimitedMotor(DcMotor motor, double bottomLimit, double topLimit, double controlAxis, double power, boolean boostOn) {
        int position = motor.getCurrentPosition();
        telemetry.addData(
                "LIMIT motor " + motor.getDeviceName() + " CTRL " + controlAxis +
                        " POS " + position + " BOTTOM " + bottomLimit + " TOP " + topLimit, "");
        if (controlAxis > 0.1 && position < topLimit) {
            if (boostOn && (gamepad1.b || gamepad1.right_bumper)){
                motor.setPower(1);
            } else {
                motor.setPower(power);
            }
        } else if (controlAxis < -0.1 && position > bottomLimit) {
            if (boostOn && (gamepad1.b || gamepad1.right_bumper)){
                motor.setPower(-1);
            } else {
                motor.setPower(-power);
            }
        } else {
            motor.setPower(0);
        }
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
            if (gamepad1.dpad_left) {
                hw.horizontalDriveMotor.setPower(-1);
            } else if (gamepad1.dpad_right) {
                hw.horizontalDriveMotor.setPower(1);
            } else {
                hw.horizontalDriveMotor.setPower(0);
            }

            if (gamepad1.left_bumper){
                hw.rightDriveMotor.setPower(gamepad1.right_stick_y);
                hw.leftDriveMotor.setPower(gamepad1.left_stick_y);
            }
            else if (gamepad1.right_bumper){
                hw.rightDriveMotor.setPower(gamepad1.right_stick_y * 0.25);
                hw.leftDriveMotor.setPower(gamepad1.left_stick_y * 0.25);
            }
            else {
                hw.rightDriveMotor.setPower(gamepad1.right_stick_y * 0.5);
                hw.leftDriveMotor.setPower(gamepad1.left_stick_y * 0.5);
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
