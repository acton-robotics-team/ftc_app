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
        DcMotor lifterMotor = hardwareMap.dcMotor.get("lifter_motor");
        waitForStart();

        while (opModeIsActive()) {
            int position = lifterMotor.getCurrentPosition();
            telemetry.addData("Encoder Position", position);
            telemetry.update();
            if (gamepad2.right_stick_y > 0.1 && position < 4.5 * Hardware.TETRIX_TICKS_PER_REVOLUTION) { //4.5 reotations
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
