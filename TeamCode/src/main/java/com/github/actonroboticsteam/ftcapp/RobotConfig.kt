package com.github.actonroboticsteam.ftcapp

import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor
import com.qualcomm.robotcore.hardware.Servo

/**
 * Robot access and abstraction.
 */
internal class RobotConfig(map: HardwareMap) {
    val rightDriveMotor: DcMotor = map.dcMotor.get("right_drive_motor")
    val leftDriveMotor: DcMotor = map.dcMotor.get("left_drive_motor")
    val lifterMotor: DcMotor = map.dcMotor.get("lifter_motor")
    val relicArmMotor: DcMotor = map.dcMotor.get("relic_arm_motor")
    val relicElbowServo: Servo = map.servo.get("relic_elbow_servo")
    val relicHandServo: Servo = map.servo.get("relic_hand_servo")
    val jewelArmServo: Servo = map.servo.get("jewel_arm_servo")
    val leftGrabberServo: Servo = map.servo.get("left_grabber_servo")
    val rightGrabberServo: Servo = map.servo.get("right_grabber_servo")
    val slideGateServo: Servo = map.servo.get("slide_gate_servo")
    val slideLifterServo: Servo = map.servo.get("slide_lifter_servo")
    val slideExtenderServo: Servo = map.servo.get("slide_extender_servo")
    val jewelColorSensor: ColorSensor = map.colorSensor.get("color_sensor")
    val ods: OpticalDistanceSensor = map.opticalDistanceSensor.get("ods")

    init {
        leftDriveMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftDriveMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        leftDriveMotor.direction = DcMotorSimple.Direction.REVERSE
        lifterMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        lifterMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        relicArmMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        relicArmMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        relicArmMotor.direction = DcMotorSimple.Direction.REVERSE
        relicHandServo.direction = Servo.Direction.REVERSE
        jewelArmServo.direction = Servo.Direction.REVERSE
        leftGrabberServo.direction = Servo.Direction.REVERSE
    }

    companion object {
        // Empirically-determined values
        val GRABBER_RELEASED = 0.5
        val GRABBER_GRABBED = 1.0
        val TETRIX_TICKS_PER_REVOLUTION = 1440
        val TETRIX_TICKS_PER_TURN_DEGREE = 2681.0 / 300.0
        val LIFTER_TOP_LIMIT = 4 * TETRIX_TICKS_PER_REVOLUTION
        val RELIC_ARM_TOP_LIMIT = 0.5 * TETRIX_TICKS_PER_REVOLUTION
        val RELIC_HAND_CLOSED = 0.7
        val RELIC_HAND_OPEN = 0.0
        val JEWEL_ARM_EXTENDED = 0.0
        val JEWEL_ARM_HALF_EXTENDED = 0.25
        val JEWEL_ARM_RETRACTED = 0.5
        val SLIDE_GATE_OPEN = 0.5
        val SLIDE_GATE_CLOSED = 0.0
    }
}

