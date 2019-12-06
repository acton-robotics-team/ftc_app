package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryLoader;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.tank.SuperiorestTankDrive;

import java.io.File;

@Autonomous
public class BaseAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SuperiorestTankDrive drive = new SuperiorestTankDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(12, 24 + 24 + 12, Math.toRadians(270)));
        waitForStart();

        Trajectory traj = TrajectoryLoader.load(new File())

        drive.followTrajectorySync(drive.trajectoryBuilder()
                // Drive to line up with foundation
                .lineTo(new Vector2d(12, 9))
                .build());
    }
}
