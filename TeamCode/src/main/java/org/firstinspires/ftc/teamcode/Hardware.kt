package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference

class Hardware(hwMap: HardwareMap) {

    val rightDrive: DcMotor = hwMap.dcMotor.get("right_motor")
    val leftDrive: DcMotor = hwMap.dcMotor.get("left_motor")
    val lifter: DcMotor = hwMap.dcMotor.get("lifter")
    val arm: DcMotor = hwMap.dcMotor.get("arm")
    val armExtender: DcMotor = hwMap.dcMotor.get("arm_extender")
    val wrist: DcMotor = hwMap.dcMotor.get("wrist")

    val leftGrabber: Servo = hwMap.servo.get("left_grabber")
    val rightGrabber: Servo = hwMap.servo.get("right_grabber")

    val imu: BNO055IMU = hwMap.get(BNO055IMU::class.java, "imu")

    val webcam: WebcamName = hwMap.get(WebcamName::class.java, "webcam")

    init {
        rightDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftDrive.direction = DcMotorSimple.Direction.REVERSE
        arm.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        arm.mode = DcMotor.RunMode.RUN_USING_ENCODER
        armExtender.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        wrist.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

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

    /**
     * Gets the current IMU heading in degrees.
     *
     * Note: this function takes a while to return.d
     */
    fun getImuHeading(): Float {
        return this.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES)
                .thirdAngle
    }

    fun withDriveMotors(fn: (motor: DcMotor) -> Unit) {
        fn(leftDrive)
        fn(rightDrive)
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
