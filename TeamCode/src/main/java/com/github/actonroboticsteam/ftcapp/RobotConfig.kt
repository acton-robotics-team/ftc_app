/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
        lifterMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        relicArmMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        relicArmMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        relicHandServo.direction = Servo.Direction.REVERSE
        jewelArmServo.direction = Servo.Direction.REVERSE
        leftGrabberServo.direction = Servo.Direction.REVERSE
    }

    companion object {
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

