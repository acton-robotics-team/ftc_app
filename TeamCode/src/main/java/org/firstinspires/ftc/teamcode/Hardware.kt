package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.BNO055IMUImpl
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.*
import edu.spa.ftclib.internal.controller.ErrorTimeThresholdFinishingAlgorithm
import edu.spa.ftclib.internal.controller.FinishableIntegratedController
import edu.spa.ftclib.internal.controller.PIDController
import edu.spa.ftclib.internal.drivetrain.HeadingableTankDrivetrain
import edu.spa.ftclib.internal.sensor.IntegratingGyroscopeSensor
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import kotlin.math.roundToInt

class Hardware(hwMap: HardwareMap, private val opMode: LinearOpMode) {
    val frontRightDrive: DcMotor = hwMap.dcMotor.get("front_right")
    val frontLeftDrive: DcMotor = hwMap.dcMotor.get("front_left")
    val backRightDrive: DcMotor = hwMap.dcMotor.get("back_right")
    val backLeftDrive: DcMotor = hwMap.dcMotor.get("back_left")

    init {
        listOf(frontLeftDrive, frontRightDrive, backRightDrive, backLeftDrive).forEach {
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
        // Set left motors in reverse
        listOf(frontLeftDrive, backLeftDrive).forEach {
            it.direction = DcMotorSimple.Direction.REVERSE
        }
    }
}
