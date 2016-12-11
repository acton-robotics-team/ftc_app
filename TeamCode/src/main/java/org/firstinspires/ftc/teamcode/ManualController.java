package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Steven on 12/5/2016.
 */

@TeleOp(name = "Activate scooper", group = "Always on")
public class ManualController extends OpMode {
    private DcMotor leftDriveMotor = null;
    private DcMotor rightDriveMotor = null;

    @Override
    public void init() {
        leftDriveMotor = hardwareMap.dcMotor.get(Hardware.ID_LEFT_DRIVE_MOTOR);
        rightDriveMotor = hardwareMap.dcMotor.get(Hardware.ID_RIGHT_DRIVE_MOTOR);
    }

    @Override
    public void start() {
        super.start();
    }


    @Override
    public void loop() {
        // Set drive motor levels
        leftDriveMotor.setPower(gamepad1.left_stick_y);
        rightDriveMotor.setPower(gamepad1.right_stick_y);

        
    }
}
