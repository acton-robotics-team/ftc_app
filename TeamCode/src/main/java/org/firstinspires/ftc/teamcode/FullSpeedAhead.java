package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by kevin on 08/01/17.
 */

@Autonomous(name = "Set motors to full speed ahead", group = "Autonomous")
public class FullSpeedAhead extends LinearOpMode {
    private Hardware robot = new Hardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.leftDriveMotor.setPower(1);
        robot.rightDriveMotor.setPower(1);

        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        robot.leftDriveMotor.setPower(0);
        robot.rightDriveMotor.setPower(0);
    }
}
