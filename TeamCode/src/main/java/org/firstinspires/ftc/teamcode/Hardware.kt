package org.firstinspires.ftc.teamcode

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

    val lifter: DcMotor = hwMap.dcMotor.get("lifter")

    val leftArmRotator: DcMotor = hwMap.dcMotor.get("arm_rotator_left")
    val rightArmRotator: DcMotor = hwMap.dcMotor.get("arm_rotator_right")
    val armExtender: DcMotor = hwMap.dcMotor.get("arm_extender")

    val boxHingeServo: Servo = hwMap.servo.get("box_hinge")
    val boxSweeper: CRServo = hwMap.crservo.get("box_sweeper")

    val markerReleaser: Servo = hwMap.servo.get("marker")

    val imu: BNO055IMUImpl = hwMap.get(BNO055IMUImpl::class.java, "imu")

    private val pid = PIDController(2.5, 0.15, 0.0).apply {
        maxErrorForIntegral = 0.002
    }
    private val controller = FinishableIntegratedController(IntegratingGyroscopeSensor(imu), pid, ErrorTimeThresholdFinishingAlgorithm(Math.PI / 12.5, 1.0))
    val drivetrain = FourWheelDriveTrain(backLeftDrive, backRightDrive, frontLeftDrive, frontRightDrive, controller)

    init {
        listOf(frontLeftDrive, frontRightDrive, backRightDrive, backLeftDrive).forEach {
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
        // Set left motors in reverse
        listOf(frontLeftDrive, backLeftDrive).forEach {
            it.direction = DcMotorSimple.Direction.REVERSE
        }
        // Let back motors use the encoder (front wheels do not have them)
        listOf(backLeftDrive, backRightDrive).forEach {
            it.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }

        listOf(leftArmRotator, rightArmRotator).forEach {
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.mode = DcMotor.RunMode.RUN_USING_ENCODER
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
        rightArmRotator.direction = DcMotorSimple.Direction.REVERSE
        boxHingeServo.direction = Servo.Direction.REVERSE

        armExtender.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        // Zero encoders
        lifter.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        lifter.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // Reset target positions of all motors (just in case)
        listOf(lifter, armExtender, leftArmRotator,
                rightArmRotator, backLeftDrive, backRightDrive).forEach {
            it.targetPosition = 0
        }

        boxSweeper.direction = DcMotorSimple.Direction.REVERSE

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
     * Negative = left turn, positive = right turn
     *
     * Note: this function takes a while to return.
     */
    fun getHeading(): Float {
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

    /**
     * Rotates the arm by X degrees from the original position of the arm
     * (aka stowed away; horizontal)
     *
     * Blocks (it will wait until the movement is complete)
     */
    fun rotateArmFromStartPosition(degrees: Float) {
        if (degrees < 0 || degrees > 180) {
            throw IllegalArgumentException("wtf are you doing that's an unsafe arm value")
        }
        val ticks = (2 * NEVEREST_40_TICKS_PER_REV * (degrees / 360)).roundToInt()

        opMode.telemetry.logEx("Moving both arm motors to $ticks ticks")

        listOf(leftArmRotator, rightArmRotator).forEach {
            it.mode = DcMotor.RunMode.RUN_TO_POSITION
            it.power = 0.1
            it.targetPosition = ticks
        }

        while (opMode.opModeIsActive() && leftArmRotator.isBusy && rightArmRotator.isBusy) {
        }
        opMode.telemetry.logEx("Finished arm rotation of $degrees degrees")
    }

    fun drive(inches: Double, speed: Double = Hardware.DRIVE_FAST) {
        backRightDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        backLeftDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        backRightDrive.mode = DcMotor.RunMode.RUN_TO_POSITION
        backLeftDrive.mode = DcMotor.RunMode.RUN_TO_POSITION

        val requiredEncoderTicks = (inches * Hardware.DRIVE_ENCODER_TICKS_PER_IN).roundToInt()

        log("Driving $inches in ($requiredEncoderTicks ticks) to end point")
        val leftEncoderTelemetry = opMode.telemetry.addData("Left drive encoder", 0)
        val rightEncoderTelemetry = opMode.telemetry.addData("Right drive encoder", 0)
        opMode.telemetry.update()

        if (inches > 0) {
            setDrivePower(speed)
        } else {
            setDrivePower(-speed)
        }
        backLeftDrive.targetPosition = requiredEncoderTicks
        backRightDrive.targetPosition = requiredEncoderTicks

        while (opMode.opModeIsActive() && backRightDrive.isBusy && backLeftDrive.isBusy) {
            leftEncoderTelemetry.setValue(backLeftDrive.currentPosition)
            rightEncoderTelemetry.setValue(backRightDrive.currentPosition)
            opMode.telemetry.update()
        }

        setDrivePower(0.0)
        backLeftDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
        backRightDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    private fun doTelemetry(drivetrain: HeadingableTankDrivetrain) {
        val pid = drivetrain.controller.algorithm as PIDController
        opMode.telemetry.clear()
        opMode.telemetry.addData("heading, target",
                drivetrain.controller.sensorValue.toString() + "," + pid.target)
        opMode.telemetry.addData("KP", pid.kp)
        opMode.telemetry.addData("KI", pid.ki)
        opMode.telemetry.addData("KD", pid.kd)
        opMode.telemetry.addData("error", pid.error)
        opMode.telemetry.addData("integral", pid.integral)
        opMode.telemetry.addData("derivative", pid.derivative)
        opMode.telemetry.update()
    }

    /**
     * The position that the rover was last set to move to. Ideally, the rover's
     * actual position is pretty close to here.
     */
    private var lastTargetedPosition: Float = 0f

    /**
     * Turns by X degrees relative to the robot's current heading
     */
    fun turn(deg: Float) {
        turnFromStart(deg + lastTargetedPosition)
    }

    fun turnImprecise(deg: Float) {
        val setTurnPower = { power: Double ->
            if (deg > 0) {
                // Turn right (clockwise)
                setRightDrivePower(-power)
                setLeftDrivePower(power)
            } else {
                // Turn left (counterclockwise)
                setRightDrivePower(power)
                setLeftDrivePower(-power)
            }
        }
        // E.g. 45 deg
        val startHeading = getHeading()
        // E.g. 45 deg + 25 deg = 65 deg
        val targetHeading = startHeading + deg
        // Drive slowly because reading the IMU is slow and takes a while
        val reachedTargetCondition: (heading: Float) -> Boolean = when {
            // Gone past target heading conditions
            deg > 0 -> { heading -> heading > targetHeading }
            else -> { heading -> heading < targetHeading }
        }

        log("Starting turn of $deg degrees from initial heading $startHeading")
        log("Target IMU heading: $targetHeading deg")

        setTurnPower(0.45)

        var heading = getHeading()
        val headingTelemetry = opMode.telemetry.addData("Current heading", heading)
        while (opMode.opModeIsActive() && !reachedTargetCondition(heading)) {
            if (Math.abs(heading - targetHeading) < 10) {
                setTurnPower(Hardware.DRIVE_SLOWEST)
            }
            heading = getHeading()
            headingTelemetry.setValue(heading)
            opMode.telemetry.update()
        }
        setDrivePower(0.0)
        log("Finished turn.")
        lastTargetedPosition += deg
    }

    /**
     * Turns from X degrees relative to the starting heading
     *
     * Right = negative, left = position
     */
    fun turnFromStart(deg: Float) {
        var rad = 0.0
        var currentHeading = getHeading() * Math.PI / 180
        while (rad >= currentHeading + Math.PI || rad <= currentHeading - Math.PI) {
            if (rad >= currentHeading + Math.PI) {
                rad -= 2 * Math.PI
            } else if (rad <= currentHeading - Math.PI) {
                rad += 2 * Math.PI
            }
        }
        rad += deg * Math.PI / 180
        log("turning to" + rad)
        drivetrain.targetHeading = rad

        while (opMode.opModeIsActive() && drivetrain.isRotating) {
            doTelemetry(drivetrain)
            drivetrain.updateHeading()
        }
        setDrivePower(0.0)
        log("finished turning")
        lastTargetedPosition = deg
    }

    private fun log(entry: String) {
        opMode.telemetry.logEx(entry)
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
        const val LIFTER_TOP_POSITION = 15000
        const val LIFTER_AUTO_DROP_DOWN_POSITION = LIFTER_TOP_POSITION
        const val LIFTER_AUTO_END_POSITION = LIFTER_BOTTOM_POSITION

        const val ARM_ROTATION_BOTTOM_LIMIT = 0
        const val ARM_ROTATION_UPPER_LIMIT = 1000

        const val ARM_TEAM_MARKER_DROP_POSITION = 767

        const val MARKER_RELEASED = 0.1
        const val MARKER_RETRACTED = 1.0

        const val OMNIWHEELS_RADIUS_IN = 2
        const val DRIVE_ENCODER_TICKS_PER_IN = NEVEREST_40_TICKS_PER_REV / (2 * Math.PI * OMNIWHEELS_RADIUS_IN)
    }
}
