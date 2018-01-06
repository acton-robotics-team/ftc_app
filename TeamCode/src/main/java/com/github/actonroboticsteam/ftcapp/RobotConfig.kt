package com.github.actonroboticsteam.ftcapp

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.roundToInt

/**
 * Robot access and abstraction.
 */
internal class RobotConfig(map: HardwareMap) {
    val rightDriveMotor: DcMotor = map.dcMotor.get("right_drive_motor")
    val leftDriveMotor: DcMotor = map.dcMotor.get("left_drive_motor")
    val lifterMotor: DcMotor = map.dcMotor.get("lifter_motor")
    val relicExtenderMotor: DcMotor = map.dcMotor.get("relic_extender_motor")
    val relicElbowServo: Servo = map.servo.get("relic_elbow_servo")
    val relicHandServo: Servo = map.servo.get("relic_hand_servo")
    val leftBottomGrabberServo: Servo = map.servo.get("left_bottom_grabber_servo")
    val rightBottomGrabberServo: Servo = map.servo.get("right_bottom_grabber_servo")
    val leftTopGrabberServo: Servo = map.servo.get("left_top_grabber_servo")
    val rightTopGrabberServo: Servo = map.servo.get("right_top_grabber_servo")


    init {
        leftDriveMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftDriveMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        leftDriveMotor.direction = DcMotorSimple.Direction.REVERSE
        lifterMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        lifterMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        relicHandServo.direction = Servo.Direction.REVERSE
        rightBottomGrabberServo.direction = Servo.Direction.REVERSE
        rightTopGrabberServo.direction = Servo.Direction.REVERSE
    }

    fun setGrabbers(pos: Double) {
        this.leftBottomGrabberServo.position = pos
        this.rightBottomGrabberServo.position = pos
        this.leftTopGrabberServo.position = pos
        this.rightTopGrabberServo.position = pos
    }

    companion object {
        // Empirically-determined values
        const val GRABBER_RELEASED = 0.5
        const val GRABBER_GRABBED = 1.0
        const val TETRIX_TICKS_PER_REVOLUTION = 1440
        const val TETRIX_TICKS_PER_TURN_DEGREE = 2681.0 / 360.0
        const val LIFTER_TOP_LIMIT = 4 * TETRIX_TICKS_PER_REVOLUTION
        const val RELIC_HAND_CLOSED = 0.7
        const val RELIC_HAND_OPEN = 0.0

    }
}

