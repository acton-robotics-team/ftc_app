package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Manual mecanum drive")
public class ManualMecanumDrive extends LinearOpMode {
    private Hardware hw;

    private void runMecanumDrive() {
        // Reset speed variables
        double LF = 0;
        double RF = 0;
        double LR = 0;
        double RR = 0;

        // Get joystick values
        double Y1 = -gamepad1.right_stick_y; // invert so up is positive
        double X1 = gamepad1.right_stick_x;
//            double Y2 = -gamepad1.left_stick_y; // Y2 is not used at present
        double X2 = gamepad1.left_stick_x;

        // Forward/back movement
        LF += Y1; RF += Y1; LR += Y1; RR += Y1;

        // Side to side movement
        LF += X1; RF -= X1; LR -= X1; RR += X1;

        // Rotation movement
        LF += X2; RF -= X2; LR += X2; RR -= X2;

        // Send values to the motors
        hw.frontLeftDrive.setPower(LF);
        hw.frontRightDrive.setPower(RF);
        hw.backLeftDrive.setPower(LR);
        hw.backRightDrive.setPower(RR);

        // Send some useful parameters to the driver station
        telemetry.addData("LF", "%.3f", LF);
        telemetry.addData("RF", "%.3f", RF);
        telemetry.addData("LR", "%.3f", LR);
        telemetry.addData("RR", "%.3f", RR);
    }

    private void runIntake() {
        hw.leftIntake.setPower(gamepad1.left_trigger);
        hw.rightIntake.setPower(gamepad1.right_trigger);
    }

    @Override
    public void runOpMode() {
        hw = new Hardware(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();

        telemetry.addData("Status", "Initialized");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            runMecanumDrive();
            runIntake();
            telemetry.update();
        }
    }
}
