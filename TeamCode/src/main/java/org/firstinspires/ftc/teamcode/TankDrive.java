package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TankDrive extends LinearOpMode {
    private Hardware hw;

    private void drivetrain() {
        hw.backLeftDrive.setPower(gamepad1.left_stick_y);
        hw.frontLeftDrive.setPower(gamepad1.left_stick_y);
        hw.backRightDrive.setPower(gamepad1.right_stick_y);
        hw.frontRightDrive.setPower(gamepad1.right_stick_y);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        hw = new Hardware(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime loopTime = new ElapsedTime();

        while (opModeIsActive()) {
            loopTime.reset();

            drivetrain();

            telemetry.addData("Run time", runtime.milliseconds());
            telemetry.addData("Loop time", loopTime.milliseconds());
            telemetry.update();
        }
    }
}
