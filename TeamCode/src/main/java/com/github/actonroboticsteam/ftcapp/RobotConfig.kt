package com.github.actonroboticsteam.ftcapp

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

/**
 * Robot access and abstraction.
 */
internal class RobotConfig(map: HardwareMap) {
    val rightDriveMotor: DcMotor = map.dcMotor.get("right_drive_motor")
    val leftDriveMotor: DcMotor = map.dcMotor.get("left_drive_motor")
    val lifterMotor: DcMotor = map.dcMotor.get("lifter_motor")
    val relicArmMotor: DcMotor = map.dcMotor.get("relic_arm_motor")
    private val relicGrabber1Servo: Servo = map.servo.get("relic_grabber_1_servo")
    private val relicGrabber2Servo: Servo = map.servo.get("relic_grabber_2_servo")
    private val leftBottomGrabberServo: Servo = map.servo.get("left_bottom_grabber_servo")
    private val rightBottomGrabberServo: Servo = map.servo.get("right_bottom_grabber_servo")
    private val leftTopGrabberServo: Servo = map.servo.get("left_top_grabber_servo")
    private val rightTopGrabberServo: Servo = map.servo.get("right_top_grabber_servo")

    init {
        leftDriveMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftDriveMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        leftDriveMotor.direction = DcMotorSimple.Direction.REVERSE
        lifterMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        lifterMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        relicGrabber1Servo.direction = Servo.Direction.REVERSE
        rightBottomGrabberServo.direction = Servo.Direction.REVERSE
        rightTopGrabberServo.direction = Servo.Direction.REVERSE
    }

    fun setGlyphGrabbers(pos: Double) {
        this.leftBottomGrabberServo.position = pos
        this.rightBottomGrabberServo.position = pos
        this.leftTopGrabberServo.position = pos
        this.rightTopGrabberServo.position = pos
    }

    fun setRelicGrabbers(pos: Double) {
        this.relicGrabber1Servo.position = pos
        this.relicGrabber2Servo.position = pos
    }

    companion object {
        // Empirically-determined values
        const val GLYPH_GRABBER_RELEASED = 0.5
        const val GLYPH_GRABBER_GRABBED = 1.0
        const val RELIC_GRABBER_RELEASED = 0.0
        const val RELIC_GRABBER_GRABBED = 1.0
        private const val TETRIX_TICKS_PER_REVOLUTION = 1440
        const val LIFTER_TOP_LIMIT = 4.0 * TETRIX_TICKS_PER_REVOLUTION
        const val RELIC_ARM_TOP_LIMIT = 0.25 * TETRIX_TICKS_PER_REVOLUTION
    }
}

