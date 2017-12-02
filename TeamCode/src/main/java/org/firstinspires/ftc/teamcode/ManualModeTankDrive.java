package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
 * - Right analog stick: relic arm motor
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

                double slideLifterDelta = 0;
                if (gamepad2.dpad_right) {
                    slideLifterDelta = 0.05;
                } else if (gamepad2.dpad_left) {
                    slideLifterDelta = -0.05;
                }
                hw.slideLifterServo.setPosition(limit(
                        hw.slideLifterServo.getPosition() + slideLifterDelta, 0, 1));

                double slideExtenderDelta = 0;
                if (gamepad2.dpad_up) {
                    slideExtenderDelta = 0.05;
                } else if (gamepad2.dpad_down) {
                    slideExtenderDelta = -0.05;
                }
                hw.slideExtenderServo.setPosition(limit(
                        hw.slideExtenderServo.getPosition() + slideExtenderDelta, 0, 1));
                if (gamepad2.y) {
                    hw.slideGateServo.setPosition(
                            // is the gate like pretty much closed? open it! otherwise close it
                            Math.abs(hw.slideGateServo.getPosition() - Hardware.SLIDE_GATE_CLOSED) <= 0.1
                                    ? Hardware.SLIDE_GATE_OPEN
                                    : Hardware.SLIDE_GATE_CLOSED);
                }

                controlLimitedMotor(
                        hw.lifterMotor,
                        0, Hardware.LIFTER_TOP_LIMIT,
                        -gamepad2.left_stick_y, 1);

                // right stick: motor for arm
                // a+right stick: servo for elbow
                // b: relic hand
                hw.relicHandServo.setPosition(limit(gamepad2.b ? 1 : 0, 0, 0.67));

                if (gamepad2.a) {
                    hw.relicArmMotor.setPower(0);
                    hw.relicElbowServo.setPosition(limit(0.05 + hw.relicElbowServo.getPosition(), 0, 1));
                } else {
                    controlLimitedMotor(
                            hw.relicArmMotor,
                            0, Hardware.RELIC_ARM_TOP_LIMIT,
                            gamepad2.right_stick_y, 0.1);
                }
                telemetry.addData("Left drive encoder value", hw.leftDriveMotor.getCurrentPosition());

            } catch (Exception e) {
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
