package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Steven on 12/5/2016.
 */
@TeleOp(name = "Manual Controller", group = "Manual")
public class ManualController extends OpMode {
    private Hardware robot = new Hardware();

    @Override
    public void init() {
    }

    @Override
    public void start() {
        super.start();

        robot.launcherMotor.setPower(0);
        robot.scooperServo.setPosition(Hardware.POS_SCOOPER_SERVO_DOWN);
    }

    /**
     * Top - launcher, scooper
     */
    private void loopTop() {
        // LAUNCHER MOTOR CONTROL
        if (gamepad2.x) {
            // Full speed
            robot.launcherMotor.setPower(1);
        } else if (gamepad2.y) {
            // Medium speed
            robot.launcherMotor.setPower(2/3);
        } else if (gamepad2.b) {
            // Slow speed
            robot.launcherMotor.setPower(1/3);
        } else if (gamepad2.a) {
            // Off
            robot.launcherMotor.setPower(0/3);
        }

        // SCOOPER SERVO CONTROL
        if (gamepad2.left_bumper) {
            robot.scooperServo.setPosition(Hardware.POS_SCOOPER_SERVO_DOWN);
        } else if (gamepad2.right_bumper) {
            robot.scooperServo.setPosition(Hardware.POS_SCOOPER_SERVO_UP);
        }
    }

    /**
     * Bottom - drive control
     */
    private void loopBottom() {
        // Set drive motor levels
        robot.leftDriveMotor.setPower(gamepad1.left_stick_y);
        robot.rightDriveMotor.setPower(gamepad1.right_stick_y);
    }

    @Override
    public void loop() {
        loopTop();
        loopBottom();
    }
}
