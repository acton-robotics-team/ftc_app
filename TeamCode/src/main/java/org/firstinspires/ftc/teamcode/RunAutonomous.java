package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

        robot.leftDriveMotor.setPower(1);
        robot.rightDriveMotor.setPower(1);

        while (opModeIsActive() && robot.floorLightSensor.getLightDetected() < 34) {
            idle();
        }

        // STEP 2 -- ROTATE TO BE PARALLEL TO WALL

        

        // STEP 3 -- MOVE TO FIRST BEACON


        // STEP 4 -- ACTIVATE BEACON


        // STEP 5 -- MOVE TO SECOND BEACON


        // STEP 6 -- ACTIVATE BEACOn
    }
}
