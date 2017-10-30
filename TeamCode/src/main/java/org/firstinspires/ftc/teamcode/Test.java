package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by nitro on 10/9/2017.
 */

@TeleOp(name = "Test")
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            int offset;
            DcMotor lifterMotor = hardwareMap.dcMotor.get("lifter_motor");


            int position = lifterMotor.getCurrentPosition();
           telemetry.addData("Encoder Position", position);
           lifterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (gamepad2.right_stick_y > 0.1 && position < -4500) { //4.5 reotations
                // Actually pushing down -- positive offset = move DOWN
                lifterMotor.setPower(1);

            } else if (gamepad2.right_stick_y < -0.1 && position > 0) {
                // Actually pushing up -- negative offset = move UP
                lifterMotor.setPower(-1);
            } else {
                lifterMotor.setPower(0);
            }

            idle();
        }
    }
}
