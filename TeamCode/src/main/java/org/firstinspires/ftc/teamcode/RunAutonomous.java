package org.firstinspires.ftc.teamcode;

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
    private static final int RED_COLOR_VALUE = 5000;
    private static final int ULTRASONIC_ERROR_MARGIN = 0;

    private boolean lineNotDetected() {
        return robot.floorLightSensor.getLightDetected() < LINE_LIGHT_THRESHOLD;
    }

    private void reverseToNextLine() {
        robot.leftDriveMotor.setPower(-1);
        robot.rightDriveMotor.setPower(-1);

        while (opModeIsActive() && lineNotDetected()) {
            idle();
        }

        robot.leftDriveMotor.setPower(0);
        robot.rightDriveMotor.setPower(0);
    }

    private void pressCorrectBeaconButton() {
        int colorSensorValue = robot.sideColorSensor.argb();
        Servo servoToMove;

        if (colorSensorValue >= RED_COLOR_VALUE) {
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

        // STEP 1 -- MOVE TO CORNER

        // Motors full throttle ahead
        robot.leftDriveMotor.setPower(1);
        robot.rightDriveMotor.setPower(1);

        // Displays readings from the ultrasonic sensor when the opMode is active
        while (opModeIsActive()) {
            telemetry.addLine(robot.sideFrontUltrasonicSensor.getUltrasonicLevel()+"");
            telemetry.addLine(robot.sideBackUltrasonicSensor.getUltrasonicLevel()+"");
        }

        // Keep moving until required distance is reached
        while (opModeIsActive() && lineNotDetected()) {
            idle();
        }

        // STEP 2 -- ROTATE TO BE PARALLEL TO WALL

        // Turning right
        robot.leftDriveMotor.setPower(1);
        robot.rightDriveMotor.setPower(-1);

        /* CODE FOR TURNING LEFT
            robot.rightDriveMotor.setPower(-1);
            robot.leftDriveMotor.setPower(1);
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
