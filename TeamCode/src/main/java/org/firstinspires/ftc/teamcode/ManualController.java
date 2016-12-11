package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Steven on 12/5/2016.
 */

@TeleOp(name = "Activate scooper", group = "Always on")
public class ManualController extends OpMode {
    private DcMotor leftDriveMotor = null;
    private DcMotor rightDriveMotor = null;
    private DcMotor launcherMotor = null;
    private Servo scooperServo = null;
    private boolean isLaunching = false;

    @Override
    public void init() {
        leftDriveMotor = hardwareMap.dcMotor.get(Hardware.ID_LEFT_DRIVE_MOTOR);
        rightDriveMotor = hardwareMap.dcMotor.get(Hardware.ID_RIGHT_DRIVE_MOTOR);
        launcherMotor = hardwareMap.dcMotor.get(Hardware.ID_LAUNCHER_MOTOR);
        scooperServo = hardwareMap.servo.get(Hardware.ID_SCOOPER_SERVO);
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

        if (gamepad2.right_bumper && gamepad2.left_bumper) {
            // Firing sequence
            telemetry.addData("Status", "PREPARE FOR DEATH");
            telemetry.update();
            isLaunching = true;

            launcherMotor.setPower(1.0); // FULL SPEED AHEAD

            // Scan servo
            if (scooperServo.getPosition() == Hardware.POS_SCOOPER_SERVO_DOWN) {
                scooperServo.setPosition(Hardware.POS_SCOOPER_SERVO_UP);
            } else if (scooperServo.getPosition() == Hardware.POS_SCOOPER_SERVO_UP) {
                scooperServo.setPosition(Hardware.POS_SCOOPER_SERVO_DOWN);
            }
        } else {
            launcherMotor.setPower(0.0);
        }
    }
}
