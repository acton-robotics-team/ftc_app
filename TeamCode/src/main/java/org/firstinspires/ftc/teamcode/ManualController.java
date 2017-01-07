package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Steven on 12/5/2016.
 */
@TeleOp(name = "Manual Controller", group = "Always on")
public class ManualController extends OpMode {
    private DcMotor leftDriveMotor = null;
    private DcMotor rightDriveMotor = null;
    private DcMotor launcherMotor = null;
    private Servo scooperServo = null;
    private Servo sweeperServo = null;

    @Override
    public void init() {
        leftDriveMotor = hardwareMap.dcMotor.get(Hardware.ID_LEFT_DRIVE_MOTOR);
        rightDriveMotor = hardwareMap.dcMotor.get(Hardware.ID_RIGHT_DRIVE_MOTOR);
        launcherMotor = hardwareMap.dcMotor.get(Hardware.ID_LAUNCHER_MOTOR);
        scooperServo = hardwareMap.servo.get(Hardware.ID_SCOOPER_SERVO);
        sweeperServo = hardwareMap.servo.get(Hardware.ID_SWEEPER_SERVO);
    }

    @Override
    public void start() {
        super.start();

        launcherMotor.setPower(0);
        scooperServo.setPosition(Hardware.POS_SCOOPER_SERVO_DOWN);
        sweeperServo.setPosition(Hardware.POS_SWEEPER_SERVO_DOWN);
    }


    private void loopTop() {
        if (gamepad2.x) {
            // Full speed
            launcherMotor.setPower(1.0);
        } else if (gamepad2.y) {
            // Medium speed
            launcherMotor.setPower(0.66);
        } else if (gamepad2.b) {
            // Slow speed
            launcherMotor.setPower(0.33);
        } else if (gamepad2.a) {
            // Off
            launcherMotor.setPower(0.0);
        }

        if (gamepad2.left_bumper) {
            sweeperServo.setPosition(Hardware.POS_SWEEPER_SERVO_UP);
        } else if (gamepad2.right_bumper) {
            sweeperServo.setPosition(Hardware.POS_SWEEPER_SERVO_DOWN);
        }
    }

    private void loopBottom() {
        // Set drive motor levels
        leftDriveMotor.setPower(gamepad1.left_stick_y);
        rightDriveMotor.setPower(gamepad1.right_stick_y);
    }

    @Override
    public void loop() {
        loopTop();
        loopBottom();
    }
}
