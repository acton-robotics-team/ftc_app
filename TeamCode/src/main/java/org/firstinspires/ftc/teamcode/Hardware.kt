package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.BNO055IMUImpl
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference

class Hardware(hwMap: HardwareMap) {
    val frontRightDrive: DcMotor = hwMap.dcMotor.get("front_right")
    val frontLeftDrive: DcMotor = hwMap.dcMotor.get("front_left")
    val backRightDrive: DcMotor = hwMap.dcMotor.get("back_right")
    val backLeftDrive: DcMotor = hwMap.dcMotor.get("back_left")

    val lifter: DcMotor = hwMap.dcMotor.get("lifter")

    val armRotatorLeft: DcMotor = hwMap.dcMotor.get("arm_rotator_left")
    val armRotatorRight: DcMotor = hwMap.dcMotor.get("arm_rotator_right")
    val armExtender: DcMotor = hwMap.dcMotor.get("arm_extender")

    val boxHingeServo1: Servo = hwMap.servo.get("box_hinge1")
    val boxHingeServo2: Servo = hwMap.servo.get("box_hinge2")
    val boxSweeper: Servo = hwMap.servo.get("box_sweeper")

    val markerReleaser: Servo = hwMap.servo.get("marker")

    val imu: BNO055IMUImpl = hwMap.get(BNO055IMUImpl::class.java, "imu")

    init {
        listOf(frontLeftDrive, frontRightDrive, backRightDrive, backLeftDrive).forEach {
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
        listOf(frontLeftDrive, backLeftDrive).forEach {
            it.direction = DcMotorSimple.Direction.REVERSE
            it.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }

        listOf(armRotatorLeft, armRotatorRight).forEach {
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.mode = DcMotor.RunMode.RUN_USING_ENCODER
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
        armRotatorLeft.direction = DcMotorSimple.Direction.REVERSE

        armExtender.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        armExtender.direction = DcMotorSimple.Direction.REVERSE

        // Zero encoders
        lifter.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        lifter.direction = DcMotorSimple.Direction.REVERSE
        lifter.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // Reset target positions of all motors (just in case)
        listOf(lifter, armExtender, armRotatorLeft, armRotatorRight, backLeftDrive, backRightDrive).forEach {
            it.targetPosition = 0
        }

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

    fun withArmRotators(action: (DcMotor) -> Unit) {
        action(armRotatorLeft)
        action(armRotatorRight)
    }

    companion object {
        const val FRONT_TETRIX_RPM = 152.0
        const val FRONT_NEVEREST_RPM = 160.0
        const val TETRIX_TO_NEVEREST_POWER = FRONT_NEVEREST_RPM / FRONT_TETRIX_RPM
        const val NEVEREST_40_TICKS_PER_REV = 1120

        const val DRIVE_SLOWEST = 0.15
        const val DRIVE_SLOW = 0.5
        const val DRIVE_FAST = 1.0

        const val LIFTER_BOTTOM_POSITION = 0
        const val LIFTER_TOP_POSITION = 14700
        const val LIFTER_AUTO_DROP_DOWN_POSITION = LIFTER_TOP_POSITION
        const val LIFTER_AUTO_END_POSITION = LIFTER_BOTTOM_POSITION

        const val ARM_GRABBING_POSITION = -600
        const val ARM_SCORING_POSITION = -1600
        const val ARM_RETRACTED = 0
        const val ARM_EXTENDED = 11253

        const val WRIST_GRABBING_POSITION = 466
        const val WRIST_SCORING_POSITION = 384

        const val MARKER_RELEASED = 0.1
        const val MARKER_RETRACTED = 1.0

        const val OMNIWHEELS_RADIUS_IN = 2
        const val DRIVE_ENCODER_TICKS_PER_IN = NEVEREST_40_TICKS_PER_REV / (2 * Math.PI * OMNIWHEELS_RADIUS_IN)
    }
}
