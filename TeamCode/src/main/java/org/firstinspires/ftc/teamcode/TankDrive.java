package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@TeleOp(name = "Tank Drive")
public class TankDrive extends LinearOpMode {
    private Hardware hw;

    private void drivetrain() {
        hw.backLeftDrive.setPower(gamepad1.left_stick_y);
        hw.frontLeftDrive.setPower(gamepad1.left_stick_y);
        hw.backRightDrive.setPower(gamepad1.right_stick_y);
        hw.frontRightDrive.setPower(gamepad1.right_stick_y);
    }

    private void runArm() {
        double armPower = 0;
        if (gamepad1.dpad_up) {
            armPower = 0.5;
        } else if (gamepad1.dpad_down) {
            armPower = -0.5;
        }
        int newPosition = (int) (hw.leftArm.getCurrentPosition() + (50 * armPower));
        for (DcMotor motor : Arrays.asList(hw.leftArm, hw.rightArm)) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.7);
            if (Math.abs(armPower) > 0.1) {
                motor.setTargetPosition(newPosition);
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        hw = new Hardware(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime loopTime = new ElapsedTime();

        hw.leftArm.setTargetPosition(0);
        hw.rightArm.setTargetPosition(0);

        while (opModeIsActive()) {
            loopTime.reset();

            drivetrain();
            runArm();

            telemetry.addData("Run time", runtime.milliseconds());
            telemetry.addData("Loop time", loopTime.milliseconds());
            telemetry.update();
        }
    }
}
