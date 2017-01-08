package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by kevin on 07/01/17.
 */

@Autonomous(name = "Run Autonomous", group = "Autonomous")
class RunAutonomous extends LinearOpMode {
    private Hardware robot = new Hardware();
    // Constants to create
    private static final int LINE_LIGHT_THRESHOLD = 0;
    private static final int RED_COLOR_VALUE =  ;
    private static final int ULTRASONIC_WALL_DISTANCE = 0;
    private static final int ULTRASONIC_ERROR_MARGIN = 0;

    private static final int SPEED = 1;

    private boolean lineNotDetected() {
        return robot.floorLightSensor.getLightDetected() < LINE_LIGHT_THRESHOLD;
    }

    private void reverseToNextLine() {
        robot.leftDriveMotor.setPower(-SPEED);
        robot.rightDriveMotor.setPower(-SPEED);

        while (opModeIsActive() && lineNotDetected()) {
            idle();
        }

        robot.leftDriveMotor.setPower(0);
        robot.rightDriveMotor.setPower(0);
    }

    private void pressCorrectBeaconButton() {
        float hsv[] = { 0F, 0F, 0F };

        Color.RGBToHSV(
                (robot.sideColorSensor.red() * 255) / 800,
                (robot.sideColorSensor.green() * 255) / 800,
                (robot.sideColorSensor.blue() * 255) / 800,
                hsv);

        Servo servoToMove;

        if (hsv[0] * 360 >= 345 || hsv[0] * 360 <= 15) {
            // Red
            servoToMove = robot.beaconRightServo;
        }
        else {
            servoToMove = robot.beaconLeftServo;
        }

        servoToMove.setPosition(Hardware.POS_BEACON_SERVO_EXTENDED);

        while (servoToMove.getPosition() != Hardware.POS_BEACON_SERVO_EXTENDED) {
            idle();
        }

        servoToMove.setPosition(Hardware.POS_BEACON_SERVO_RETRACTED);

        while (servoToMove.getPortNumber() != Hardware.POS_BEACON_SERVO_RETRACTED) {
            idle();
        }
    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        // STEP 1 -- MOVE TO CORNER

        // Motors full throttle ahead
        robot.leftDriveMotor.setPower(SPEED);
        robot.rightDriveMotor.setPower(SPEED);

        // Keep moving until required distance is reached
        while (opModeIsActive() &&
                robot.sideFrontUltrasonicSensor.getUltrasonicLevel() > ULTRASONIC_WALL_DISTANCE) {
            idle();
        }

        // STEP 2 -- ROTATE TO BE PARALLEL TO WALL

        // Turning right
        robot.leftDriveMotor.setPower(SPEED);
        robot.rightDriveMotor.setPower(-SPEED);

        /* CODE FOR TURNING LEFT
            robot.rightDriveMotor.setPower(-SPEED);
            robot.leftDriveMotor.setPower(SPEED);
            */

        while (opModeIsActive() &&
                Math.abs(
                        robot.sideFrontUltrasonicSensor.getUltrasonicLevel() -
                                robot.sideBackUltrasonicSensor.getUltrasonicLevel())
                        >= ULTRASONIC_ERROR_MARGIN) {
            idle();
        }

        // STEP 3 -- MOVE TO FIRST BEACON

        reverseToNextLine();

        // STEP 4 -- ACTIVATE BEACON

        pressCorrectBeaconButton();

        // STEP 5 -- MOVE TO SECOND BEACON

        reverseToNextLine();

        // STEP 6 -- ACTIVATE BEACON

        pressCorrectBeaconButton();
    }
}
