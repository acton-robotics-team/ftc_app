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
    private val frontRightDrive: DcMotor = hwMap.dcMotor.get("front_right")
    private val frontLeftDrive: DcMotor = hwMap.dcMotor.get("front_left")
    val backRightDrive: DcMotor = hwMap.dcMotor.get("back_right")
    val backLeftDrive: DcMotor = hwMap.dcMotor.get("back_left")
    val lifter: DcMotor = hwMap.dcMotor.get("lifter")
    val arm: DcMotor = hwMap.dcMotor.get("arm")
    val armExtender: DcMotor = hwMap.dcMotor.get("arm_extender")
    val wrist: DcMotor = hwMap.dcMotor.get("wrist")

    val leftGrabber: Servo = hwMap.servo.get("left_grabber")
    val rightGrabber: Servo = hwMap.servo.get("right_grabber")

    val imu: BNO055IMU = hwMap.get(BNO055IMU::class.java, "imu")

    val webcam: WebcamName = hwMap.get(WebcamName::class.java, "webcam")

    init {
        frontRightDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        frontLeftDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backRightDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backLeftDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        frontLeftDrive.direction = DcMotorSimple.Direction.REVERSE
        backLeftDrive.direction = DcMotorSimple.Direction.REVERSE
        arm.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        arm.mode = DcMotor.RunMode.RUN_TO_POSITION

        armExtender.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        armExtender.mode = DcMotor.RunMode.RUN_USING_ENCODER
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

    fun setDrivePower(power: Double) {
        setLeftDrivePower(power)
        setRightDrivePower(power)
    }

    fun setLeftDrivePower(power: Double) {
        frontLeftDrive.power = power
        backLeftDrive.power = TETRIX_TO_NEVEREST_POWER * power
    }

    fun setRightDrivePower(power: Double) {
        frontRightDrive.power = power
        backRightDrive.power = TETRIX_TO_NEVEREST_POWER * power
    }

    companion object {
        const val FRONT_TETRIX_RPM = 152.0
        const val FRONT_NEVEREST_RPM = 160.0
        const val TETRIX_TO_NEVEREST_POWER = FRONT_NEVEREST_RPM / FRONT_TETRIX_RPM

        const val DRIVE_SLOWEST = 0.35
        const val DRIVE_SLOW = 0.5
        const val DRIVE_FAST = 1.0

        const val GRABBER_GRABBED = 1.0
        const val GRABBER_RELEASED = 0.0

        const val LIFTER_BOTTOM_POSITION = 0
        const val LIFTER_TOP_POSITION = 13870
        const val LIFTER_AUTO_DROP_DOWN_POSITION = LIFTER_TOP_POSITION
        const val LIFTER_AUTO_END_POSITION = LIFTER_BOTTOM_POSITION

        const val ARM_DOWN = 0
        const val ARM_HALF_UP = -500
        const val ARM_UP = -1800
        const val ARM_RETRACTED = 0
        const val ARM_EXTENDED = 11253

        const val DRIVE_ENCODER_TICKS_PER_CM = 1000.0 / 27
    }
}
