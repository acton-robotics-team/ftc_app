package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import edu.spa.ftclib.internal.controller.ErrorTimeThresholdFinishingAlgorithm
import edu.spa.ftclib.internal.controller.FinishableIntegratedController
import edu.spa.ftclib.internal.controller.PIDController
import edu.spa.ftclib.internal.drivetrain.HeadingableTankDrivetrain
import edu.spa.ftclib.internal.sensor.IntegratingGyroscopeSensor

@Autonomous(name = "Test PID opmode")
class TestPid : LinearOpMode() {
    override fun runOpMode() {
        val hw = Hardware(hardwareMap)
        val pid = PIDController(1.5, 0.05, 0.0)
        pid.maxErrorForIntegral = 0.002

        val controller = FinishableIntegratedController(IntegratingGyroscopeSensor(hw.imu), pid, ErrorTimeThresholdFinishingAlgorithm(Math.PI / 25, 1.0))
        val drivetrain = FourWheelDriveTrain(hw.backLeftDrive, hw.backRightDrive, hw.frontLeftDrive, hw.frontRightDrive, controller)

        waitForStart()

        drivetrain.targetHeading = -Math.PI / 2
        while (opModeIsActive()) {
            telemetry.addData("Is rotating?", drivetrain.isRotating)
            doTelemetry(drivetrain)
            drivetrain.updateHeading()
        }
    }

    private fun doTelemetry(drivetrain: HeadingableTankDrivetrain) {
        val pid = drivetrain.controller.algorithm as PIDController
        telemetry.addData("heading, target",
                drivetrain.controller.sensorValue.toString() + "," + pid.target)
        telemetry.addData("KP", pid.kp)
        telemetry.addData("KI", pid.ki)
        telemetry.addData("KD", pid.kd)
        telemetry.addData("error", pid.error)
        telemetry.addData("integral", pid.integral)
        telemetry.addData("derivative", pid.derivative)
        telemetry.addData("random", Math.random())
        telemetry.update()
    }
}
