package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by nitro on 10/9/2017.
 */

@TeleOp(name = "Test")
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        OpticalDistanceSensor ods = hardwareMap.opticalDistanceSensor.get("optical_dist");
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("ODS", ods.getLightDetected());
            telemetry.update();
            idle();
        }
    }
}
