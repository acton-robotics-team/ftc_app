package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveREVOptimized;

@Autonomous(name = "Fallback midline autonomous")
public class FallbackMidlineAutonomous extends LinearOpMode {
    private MecanumDriveREVOptimized drive;

    private void driveDistance(double in) {
        double initialDistanceIn = drive.getWheelPositions().get(0);
        if (in > 0) {
            drive.setMotorPowers(0.5, 0.5, 0.5, 0.5);
            while (opModeIsActive() && drive.getWheelPositions().get(0) < initialDistanceIn + in) {
            }
        } else {
            drive.setMotorPowers(-0.5, -0.5, -0.5, -0.5);
            while (opModeIsActive() && drive.getWheelPositions().get(0) > initialDistanceIn + in) {
            }
        }
        drive.setMotorPowers(0, 0, 0, 0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDriveREVOptimized(hardwareMap);

        waitForStart();

        driveDistance(48);
    }
}
