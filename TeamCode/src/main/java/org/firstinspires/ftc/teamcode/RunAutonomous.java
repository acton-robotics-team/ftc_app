package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by kevin on 07/01/17.
 */

@Autonomous(name = "Run Autonomous", group = "Autonomous")
public class RunAutonomous extends LinearOpMode {
    private Hardware robot = new Hardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.leftDriveMotor.setPower(1);
        robot.rightDriveMotor.setPower(1);

        while (opModeIsActive() && robot.floorLightSensor.getLightDetected() < 34) {
            idle();
        }

    }
}
