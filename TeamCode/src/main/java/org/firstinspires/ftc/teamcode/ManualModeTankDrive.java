package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.PrintWriter;
import java.io.StringWriter;

/**
 * Created by kevinliu who writes tehe mstdisgusting code ew
 * on 10/9/2017.
 * - kuneel sharpedo
 * <p>
 * Controls:
 * <p>
 * GAMEPAD 1
 * - Left bumper: turbo (=1)
 * - Right bumper: slow (=0.125)
 * - Right/left analog sticks: drive motors
 * GAMEPAD 2
 * - Left trigger: grabber servos
 * - Right trigger: relic hand servo
 * - Up/down dpad: extend/retract slide (slide lifter servo)
 * - Left/right dpad: lift/lower slide (slide extender servo)
 * - Y: toggle slide gate servo
 * - Left analog stick: lifter motor
 * - NOT A && Right analog stick: relic arm motor
 * - A && right analog stick: relic elbow servo
 * - B: relic hand servo
 */
@TeleOp(name = "Manual: tank drive")
public class ManualModeTankDrive extends LinearOpMode {
    private class ToggleButtonServo {
        final Servo servo;
        final double startingPosition;
        final double pos2;
        boolean isPreviouslyPressed = false;

        ToggleButtonServo(Servo servo, double startingPosition, double pos2) {
            this.servo = servo;
            this.startingPosition = startingPosition;
            this.pos2 = pos2;
            servo.setPosition(startingPosition);
        }

        /**
         * Run this method on every tick with the new isPressed to update the position of the servo.
         * @param isPressed Whether or not the controlling button for this servo is pressed now
         */
        void updatePosition(boolean isPressed) {
            if (isPressed && !isPreviouslyPressed) {
                double newPosition = servo.getPosition() == startingPosition ? pos2 : startingPosition;
                telemetry.addData("Changing servo position to", newPosition);
                servo.setPosition(newPosition);
                isPreviouslyPressed = true;
            } else if (!isPressed && isPreviouslyPressed) {
                isPreviouslyPressed = false;
            }
        }
    }

    private double limit(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    private void controlLimitedMotor(DcMotor motor, double bottomLimit, double topLimit, double controlAxis, double power) {
        int position = motor.getCurrentPosition();
        telemetry.addLine("LIMITED MOTOR");
        telemetry.addData("Control axis", controlAxis);
        telemetry.addData("Position", position);
        telemetry.addData("Bottom limit", bottomLimit);
        telemetry.addData("Top limit", topLimit);
        if (controlAxis > 0.1 && position < topLimit) {
            telemetry.addLine("LIMIT: moving up");
            motor.setPower(power);
        } else if (controlAxis < -0.1 && position > bottomLimit) {
            telemetry.addLine("LIMIT: moving down");
            motor.setPower(-power);
        } else {
            telemetry.addLine("LIMIT: no control input");
            motor.setPower(0);
        }
    }

    private void fineControlServo(Servo servo, double bottomLimit, double topLimit, boolean isUp, boolean isDown) {
        double delta = 0;
        if (isUp) {
            delta = 0.05;
        } else if (isDown) {
            delta = -0.05;
        }
        // freaking servo api returns NaN when no value known
        double position = Double.isNaN(servo.getPosition()) ? bottomLimit : servo.getPosition();
        telemetry.addLine("CONTROL SERVO");
        telemetry.addData("Position", position);
        telemetry.addData("Bottom limit", bottomLimit);
        telemetry.addData("Top limit", topLimit);
        telemetry.addData("Is up?", isUp);
        telemetry.addData("Is down?", isDown);
        telemetry.addData("Adding delta", delta);
        servo.setPosition(limit(
                position + delta, bottomLimit, topLimit));
    }

    @Override
    public void runOpMode() {
        Hardware hw = new Hardware(hardwareMap);
        ToggleButtonServo slideGateToggle = new ToggleButtonServo(
                hw.slideGateServo, Hardware.SLIDE_GATE_CLOSED, Hardware.SLIDE_GATE_OPEN);
        ToggleButtonServo relicHandToggle = new ToggleButtonServo(
                hw.relicHandServo, Hardware.RELIC_HAND_CLOSED, Hardware.RELIC_HAND_OPEN);

        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
            try {
                // Gamepad 1
                double turbo = 0.25;
                if (gamepad1.left_bumper) {
                    turbo = 1;
                } else if (gamepad1.right_bumper) {
                    turbo = 0.125;
                }

                hw.rightDriveMotor.setPower(gamepad1.right_stick_y * turbo);
                hw.leftDriveMotor.setPower(gamepad1.left_stick_y * turbo);

                // Gamepad 2
                hw.leftGrabberServo.setPosition(limit(gamepad2.left_trigger, Hardware.GRABBER_RELEASED, Hardware.GRABBER_GRABBED));
                hw.rightGrabberServo.setPosition(limit(gamepad2.left_trigger, Hardware.GRABBER_RELEASED, Hardware.GRABBER_GRABBED));

                fineControlServo(hw.slideLifterServo, 0, 1, gamepad2.dpad_left, gamepad2.dpad_right);
                fineControlServo(hw.slideExtenderServo, 0, 1, gamepad2.dpad_up, gamepad2.dpad_down);

                slideGateToggle.updatePosition(gamepad2.y);
                telemetry.addData("Slide gate servo position", hw.slideGateServo.getPosition());

                controlLimitedMotor(
                        hw.lifterMotor,
                        0, Hardware.LIFTER_TOP_LIMIT,
                        -gamepad2.left_stick_y, 1);

                // right stick: motor for arm
                // a+right stick: servo for elbow
                // b: relic hand
                relicHandToggle.updatePosition(gamepad2.b);
                telemetry.addData("Relic hand position", hw.relicHandServo.getPosition());

                if (gamepad2.a) {
                    hw.relicArmMotor.setPower(0);
                    fineControlServo(hw.relicElbowServo, 0, 1, gamepad2.right_stick_y > 0, gamepad2.right_stick_y < 0);
                } else {
                    controlLimitedMotor(
                            hw.relicArmMotor,
                            0, Hardware.RELIC_ARM_TOP_LIMIT,
                            gamepad2.right_stick_y, 0.3);
                }
                telemetry.addData("Left drive encoder value", hw.leftDriveMotor.getCurrentPosition());
            } catch (Exception e) {
                // Global exception handler to get backtrace
                StringWriter sw = new StringWriter();
                PrintWriter pw = new PrintWriter(sw);
                e.printStackTrace(pw);
                telemetry.addLine(sw.toString()); // stack trace as a string
            } finally {
                telemetry.update();
                idle();
            }
        }
    }
}
