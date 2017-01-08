package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by kevin on 07/01/17.
 */

@Autonomous(name = "Run Autonomous", group = "Autonomous")
class RunAutonomous extends LinearOpMode {
    private Hardware robot = new Hardware();

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
        while (opModeIsActive() && robot.sideFrontUltrasonicSensor.getUltrasonicLevel() < 34) {
            idle();
        }

        // STEP 2 -- ROTATE TO BE PARALLEL TO WALL

        static int MOE = ;
        while(opModeIsActive() && Math.abs(robot.sideFrontUltrasonicSensor.getUltrasonicLevel()-robot.sideBackUltrasonicSensor.getUltrasonicLevel() ) >= MOE ) {
            // TURNING RIGHT
            robot.leftDriveMotor.setPower(1);
            robot.rightDriveMotor.setPower(-1);

            /* CODE FOR TURNING LEFT
            robot.rightDriveMotor.setPower(-1);
            robot.leftDriveMotor.setPower(1);
            */
        }

        // STEP 3 -- MOVE TO FIRST BEACON


        // STEP 4 -- ACTIVATE BEACON


        // STEP 5 -- MOVE TO SECOND BEACON

        static int WAITFORSECONDBEACON = 1000;

        robot.leftDriveMotor.setPower(1);
        robot.rightDriveMotor.setPower(1);
        Thread.sleep(WAITFORSECONDBEACON);


        // STEP 6 -- ACTIVATE BEACON
    }
}
