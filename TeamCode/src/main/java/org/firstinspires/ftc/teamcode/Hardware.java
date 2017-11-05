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
    static final int TETRIX_TICKS_PER_REVOLUTION = 1440;

    final DcMotor rightDriveMotor;
    final DcMotor leftDriveMotor;
    final DcMotor lifterMotor;
    final DcMotor relicArmMotor;
    final Servo relicHandServo;
    final Servo leftGrabberServo;
    final Servo rightGrabberServo;
    /* Constructor */
    Hardware(HardwareMap map) {
        rightDriveMotor = map.dcMotor.get("right_drive_motor");
        rightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDriveMotor = map.dcMotor.get("left_drive_motor");
        lifterMotor = map.dcMotor.get("lifter_motor");
        lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        relicArmMotor = map.dcMotor.get("relic_arm_motor");
        relicHandServo = map.servo.get("relic_hand_servo");
        relicHandServo.setDirection(Servo.Direction.REVERSE);
        leftGrabberServo = map.servo.get("left_grabber_servo");
        rightGrabberServo = map.servo.get("right_grabber_servo");
        rightGrabberServo.setDirection(Servo.Direction.REVERSE);
    }
}

