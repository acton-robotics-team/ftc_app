package org.firstinspires.ftc.teamcode.external

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import edu.spa.ftclib.internal.controller.ErrorTimeThresholdFinishingAlgorithm
import edu.spa.ftclib.internal.controller.FinishableIntegratedController
import edu.spa.ftclib.internal.controller.PIDController
import edu.spa.ftclib.internal.sensor.IntegratingGyroscopeSensor
import edu.spa.ftclib.util.PIDTuner
import org.firstinspires.ftc.teamcode.FourWheelDriveTrain
import org.firstinspires.ftc.teamcode.Hardware

/**
 * Created by Gabriel on 2018-01-07.
 */

@TeleOp(name = "Pushbot PID Tuner", group = "sample")
class PushbotPIDTuner : LinearOpMode() {
    private var tuner: PIDTuner? = null
    /**
     * User defined loop method
     *
     *
     * This method will be called repeatedly in a loop while this op mode is running
     */
    override fun runOpMode() {
        val hw = Hardware(hardwareMap, this)
        val pid = PIDController(1.5, 0.05, 0.0)
        pid.maxErrorForIntegral = 0.002

        val controller = FinishableIntegratedController(IntegratingGyroscopeSensor(hw.imu), pid, ErrorTimeThresholdFinishingAlgorithm(Math.PI / 25, 1.0))
        val drivetrain = FourWheelDriveTrain(hw.backLeftDrive, hw.backRightDrive, hw.frontLeftDrive, hw.frontRightDrive, controller)

        tuner = PIDTuner(drivetrain, controller.algorithm as PIDController, gamepad1, telemetry)

        waitForStart()

        while (opModeIsActive()) {
            tuner!!.update()
        }
    }
}
