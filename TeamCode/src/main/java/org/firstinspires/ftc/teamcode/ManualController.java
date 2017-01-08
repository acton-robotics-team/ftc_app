package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Steven on 12/5/2016.
 */
@TeleOp(name = "Manual Controller", group = "Manual")
class ManualController extends OpMode {
    private Hardware robot = new Hardware();
    private boolean isLauncherOn = false;
    private boolean isWaitingForLeftBeaconServo = true;
    private boolean isWaitingForRightBeaconServo = true;

    @Override
    public void init() {
        robot.init(hardwareMap);
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
            robot.launcherMotor.setPower(isLauncherOn ? 0 : 1);
            isLauncherOn = !isLauncherOn;
        }

        // SCOOPER SERVO CONTROL
        if (gamepad2.left_bumper) {
            robot.scooperServo.setPosition(Hardware.POS_SCOOPER_SERVO_DOWN);
        } else if (gamepad2.right_bumper) {
            robot.scooperServo.setPosition(Hardware.POS_SCOOPER_SERVO_UP);
        }

        // Launcher servo control
        if (gamepad2.dpad_left) {
            robot.launcherServo.setPosition(Hardware.POS_LAUNCHER_SERVO_DOWN);
        } else if (gamepad2.dpad_right) {
            robot.launcherServo.setPosition(Hardware.POS_LAUNCHER_SERVO_UP);
        }

        // SWEEPER MOTOR
        if (gamepad2.y) {
            // Medium speed
            robot.sweeperMotor.setPower(1);
        } else if (gamepad2.b) {
            // Slow speed
            robot.sweeperMotor.setPower(.5);
        } else if (gamepad2.a) {
            // Off
            robot.sweeperMotor.setPower(0);
        }
    }

    /**
     * Bottom - drive control
     */
    private void loopBottom() {
        // Set drive motor levels
        robot.leftDriveMotor.setPower(gamepad1.left_stick_y);
        robot.rightDriveMotor.setPower(gamepad1.right_stick_y);

        if (gamepad1.b) {
            robot.beaconLeftServo.setPosition(Hardware.POS_BEACON_SERVO_EXTENDED);
            isWaitingForLeftBeaconServo = true;
        } else if (gamepad1.a) {
            robot.beaconRightServo.setPosition(Hardware.POS_BEACON_SERVO_EXTENDED);
            isWaitingForRightBeaconServo = true;
        }

        if (isWaitingForLeftBeaconServo &&
                robot.beaconLeftServo.getPosition() == Hardware.POS_BEACON_SERVO_EXTENDED) {
            isWaitingForLeftBeaconServo = false;
            robot.beaconLeftServo.setPosition(Hardware.POS_BEACON_SERVO_RETRACTED);
        }
        else if (isWaitingForRightBeaconServo &&
                robot.beaconRightServo.getPosition() == Hardware.POS_BEACON_SERVO_EXTENDED) {
            isWaitingForRightBeaconServo = false;
            robot.beaconRightServo.setPosition(Hardware.POS_BEACON_SERVO_RETRACTED);
        }
    }

    @Override
    public void loop() {
        telemetry.addLine(""+(robot.launcherServo.getPosition()));
        loopTop();
        loopBottom();
    }
}
