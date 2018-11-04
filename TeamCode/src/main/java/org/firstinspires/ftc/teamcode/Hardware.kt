package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName

class Hardware(hwMap: HardwareMap) {

    val rightDrive: DcMotor = hwMap.dcMotor.get("right_motor")
    val leftDrive: DcMotor = hwMap.dcMotor.get("left_motor")
    val lifter: DcMotor = hwMap.dcMotor.get("lifter")
//    val arm: DcMotor = hwMap.dcMotor.get("arm")
    val arm: DcMotor = hwMap.dcMotor.get("arm_extender")

    val leftGrabber: Servo = hwMap.servo.get("left_grabber")
    val rightGrabber: Servo = hwMap.servo.get("right_grabber")

    val imu: BNO055IMU = hwMap.get(BNO055IMU::class.java, "imu")

    val camera: WebcamName = hwMap.get(WebcamName::class.java, "webcam")

    init {
        rightDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftDrive.direction = DcMotorSimple.Direction.REVERSE
        arm.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        rightGrabber.direction = Servo.Direction.REVERSE
        leftGrabber.scaleRange(0.0, 0.9)
        rightGrabber.scaleRange(0.0, 0.9)

        // Zero encoders
        lifter.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        lifter.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val imuParams = BNO055IMU.Parameters().apply {
            angleUnit = BNO055IMU.AngleUnit.DEGREES
            accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
            calibrationDataFile = "BNO055IMUCalibration.json" // requires calibration OpMode data
            loggingEnabled = true
            loggingTag = "IMU"
            // The following line would disable acceleration integration.
            // accelerationIntegrationAlgorithm = JustLoggingAccelerationIntegrator()
        }
        imu.initialize(imuParams)
    }

    companion object {
        const val DRIVE_SLOWEST = 0.35
        const val DRIVE_SLOW = 0.5
        const val DRIVE_FAST = 1.0

        const val LIFTER_BOTTOM_POSITION = 0
        const val LIFTER_TOP_POSITION = 5200

        const val ARM_DOWN_POSITION = 0 // placeholder

        const val DRIVE_ENCODER_TICKS_PER_CM = 1000.0 / 27
    }
}
