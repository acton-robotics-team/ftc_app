package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveREVOptimized;

public abstract class BaseFallbackFoundationAutonomous extends LinearOpMode {
    private MecanumDriveREVOptimized drive;
    private BaseAutonomous.Alliance alliance;

    public BaseFallbackFoundationAutonomous(BaseAutonomous.Alliance alliance) {
        this.alliance = alliance;
    }

    private Pose2d mirror(int redX, int redY, double redHeading) {
        if (alliance == BaseAutonomous.Alliance.RED) {
            return new Pose2d(redX, redY, redHeading);
        } else {
            return new Pose2d(redX, -redY, redHeading + Math.PI);
        }
    }

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
        Hardware hw = new Hardware(hardwareMap);
        drive = new MecanumDriveREVOptimized(hardwareMap);


        waitForStart();
    }
}
