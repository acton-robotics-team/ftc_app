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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Hardware access and abstraction.
 */
final class Hardware
{
    static final double GRABBER_RELEASED = 0.5;
    static final double GRABBER_GRABBED = 1;
    static final int TETRIX_TICKS_PER_REVOLUTION = 1440;
    static final double TETRIX_TICKS_PER_TURN_DEGREE = 2681d / 300d;
    static final int LIFTER_TOP_LIMIT = 4 * TETRIX_TICKS_PER_REVOLUTION;
    static final double RELIC_ARM_TOP_LIMIT = 0.5 * TETRIX_TICKS_PER_REVOLUTION;
    static final double JEWEL_ARM_EXTENDED = 0;
    static final double JEWEL_ARM_HALF_EXTENDED = 0.25;
    static final double JEWEL_ARM_RETRACTED = 0.5;
    static final double SLIDE_GATE_OPEN = 0.5;
    static final double SLIDE_GATE_CLOSED = 0;

    final DcMotor rightDriveMotor;
    final DcMotor leftDriveMotor;
    final DcMotor lifterMotor;
    final DcMotor relicArmMotor;
    final Servo relicElbowServo;
    final Servo relicHandServo;
    final Servo jewelArmServo;
    final Servo leftGrabberServo;
    final Servo rightGrabberServo;
    final Servo slideGateServo;
    final Servo slideLifterServo;
    final Servo slideExtenderServo;
    final ColorSensor jewelColorSensor;
    final OpticalDistanceSensor ods;
    /* Constructor */
    Hardware(HardwareMap map) {
        rightDriveMotor = map.dcMotor.get("right_drive_motor");
        leftDriveMotor = map.dcMotor.get("left_drive_motor");
        leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lifterMotor = map.dcMotor.get("lifter_motor");
        lifterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        relicArmMotor = map.dcMotor.get("relic_arm_motor");
        relicArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relicArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        relicElbowServo = map.servo.get("relic_elbow_servo");
        relicHandServo = map.servo.get("relic_hand_servo");
        relicHandServo.setDirection(Servo.Direction.REVERSE);
        jewelArmServo = map.servo.get("jewel_arm_servo");
        jewelArmServo.setDirection(Servo.Direction.REVERSE);
        leftGrabberServo = map.servo.get("left_grabber_servo");
        leftGrabberServo.setDirection(Servo.Direction.REVERSE);
        rightGrabberServo = map.servo.get("right_grabber_servo");
        slideGateServo = map.servo.get("slide_gate_servo");
        slideLifterServo = map.servo.get("slide_lifter_servo");
        slideExtenderServo = map.servo.get("slide_extender_servo");
        jewelColorSensor = map.colorSensor.get("color_sensor");
        ods = map.opticalDistanceSensor.get("ods");
    }
}

