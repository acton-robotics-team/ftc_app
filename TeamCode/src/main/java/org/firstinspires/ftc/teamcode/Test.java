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
    public void controlLimitedMotor(DcMotor motor, double bottomLimit, double topLimit, double controlAxis) {
        int position = motor.getCurrentPosition();
        telemetry.addData("LIMIT motor " + controlAxis + " " + position + " " + bottomLimit + " " + topLimit, "");
        if (controlAxis > 0.1 && position < topLimit) {
            motor.setPower(1);
        } else if (controlAxis < -0.1 && position > bottomLimit) {
            motor.setPower(-1);
        } else {
            motor.setPower(0);
        }
    }

    @Override
    public void runOpMode() {
        DcMotor lifterMotor = hardwareMap.dcMotor.get("lifter_motor");
        waitForStart();

        while (opModeIsActive()) {
            controlLimitedMotor(
                    lifterMotor,
                    0, 4.5 * Hardware.TETRIX_TICKS_PER_REVOLUTION,
                    gamepad2.left_stick_y);
            telemetry.update();
            idle();
        }
    }
}
