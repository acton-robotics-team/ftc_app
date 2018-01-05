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
//    val slideGateServo: Servo = map.servo.get("slide_gate_servo")   removed sensor bar
//    val slideLifterServo: Servo = map.servo.get("slide_lifter_servo")
//    val slideExtenderServo: Servo = map.servo.get("slide_extender_servo")
//    val jewelColorSensor: ColorSensor = map.colorSensor.get("color_sensor")
//    val ods: OpticalDistanceSensor = map.opticalDistanceSensor.get("ods")


    init {
        leftDriveMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftDriveMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        leftDriveMotor.direction = DcMotorSimple.Direction.REVERSE
        lifterMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        lifterMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        relicHandServo.direction = Servo.Direction.REVERSE
        leftBottomGrabberServo.direction = Servo.Direction.REVERSE
        leftTopGrabberServo.direction= Servo.Direction.REVERSE
    }

    companion object {
        // Empirically-determined values
        const val GRABBER_RELEASED = 0.5
        const val GRABBER_GRABBED = 1.0
        const val TETRIX_TICKS_PER_REVOLUTION = 1440
        const val TETRIX_TICKS_PER_TURN_DEGREE = 2681.0 / 300.0
        const val LIFTER_TOP_LIMIT = 4 * TETRIX_TICKS_PER_REVOLUTION
        const val RELIC_ARM_TOP_LIMIT = 0.8 * TETRIX_TICKS_PER_REVOLUTION
        const val RELIC_HAND_CLOSED = 0.7
        const val RELIC_HAND_OPEN = 0.0
        const val JEWEL_ARM_EXTENDED = 0.58
        const val JEWEL_ARM_HALF_EXTENDED = 0.31
        const val JEWEL_ARM_RETRACTED = 0.0
        const val SLIDE_GATE_OPEN = 1.0
        const val SLIDE_GATE_CLOSED = 0.0
        const val SLIDE_LIFTER_UP = 0.0
        const val SLIDE_LIFTER_RETRACTED = 1.0
        const val SLIDE_LIFTER_ANGLED_DOWN = 180.0 / 255.0
    }
}

