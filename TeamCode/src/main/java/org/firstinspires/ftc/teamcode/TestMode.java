package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by nitro on 10/9/2017.
 */

@TeleOp(name = "Test")
public class TestMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        ColorSensor jewelColorSensor = hardwareMap.colorSensor.get("jewelColorSensor");
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Blue", jewelColorSensor.blue());
            telemetry.addData("Red", jewelColorSensor.red());
            telemetry.update();
            idle();
        }
    }
}
