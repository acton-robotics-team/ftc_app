package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

/**
 * Created by nitro on 10/9/2017.
 */

@TeleOp(name = "Manual program")
public class Manual extends LinearOpMode {
    @Override
    public void runOpMode() {
        Hardware hw = new Hardware(hardwareMap);

        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
            hw.rightDriveMotor.setPower(gamepad1.right_stick_y);
            hw.leftDriveMotor.setPower(gamepad1.left_stick_y);

            hw.servo.setPosition(gamepad2.right_trigger);

            OpticalDistanceSensor sensor = hardwareMap.opticalDistanceSensor.get("sensor");
            telemetry.addLine(String.valueOf(sensor.getLightDetected()));
            telemetry.update();
            idle();
        }
    }
}
