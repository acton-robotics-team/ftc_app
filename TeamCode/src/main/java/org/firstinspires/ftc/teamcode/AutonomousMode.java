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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.concurrent.FutureTask;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

@Autonomous(name = "Autonomous program")
public class AutonomousMode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private void log(String text) {
        String str = "[" + runtime.time(TimeUnit.SECONDS) + "] " + text;
        telemetry.addLine(str);
        telemetry.update();
    }

    private void sleepSync() throws OpModeStoppedException {
        if (!opModeIsActive())
            throw new OpModeStoppedException();
        else
            idle();
    }

    private void turn(Hardware hw, int degrees) throws OpModeStoppedException {
        int encoderTicks = degrees * Hardware.TETRIX_TICKS_PER_TURN_DEGREE;
        hw.leftDriveMotor.setTargetPosition(encoderTicks);
        hw.leftDriveMotor.setPower(0.5);
        hw.rightDriveMotor.setPower(-0.5);
        while (hw.leftDriveMotor.isBusy()) {
            sleepSync();
        }
        hw.leftDriveMotor.setPower(0);
        hw.rightDriveMotor.setPower(0);
    }

    private RelicRecoveryVuMark detectPictogram() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AcdD/rP/////AAAAGQcYKmwTDk0lulf4t6n2JsQiodu68wCwukVguR/SeZyNkVD0OnUmmSWSrpM2jXTVVNorEhJRyV08URkTRak94XQN8/jPzVxzuOLCQ8VR8uYKuP/JoovnJM2MC3Pc1KvLlrLwWrL4185vpVaQMLRmvCkzNH+lyoEusMC7vwT4ayI6I22ceFumQuAubLp8APiT3omF4KG6W/lqNyJukt9YHgBYO/JJRVPfZg04LEhwFMixYOXfh+moWdf8zCMj+V7GUfH7Q7OGM0jobzVrg0uYboA2nrJBRjQS6j2eGoXX4yRwhmeVLVtBuklgw+n3qXgQ+OX9Lp48xNIApOByAlAhU117gDYYwE5NQ8ADKvtgupKd";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();
        while (true) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                return vuMark;
            } else {
                log("VuMark not visible");
                idle();
            }
        }
    }

    @Override
    public void runOpMode() {
        Hardware hw = new Hardware(hardwareMap);

        hw.leftGrabberServo.setPosition(Hardware.GRABBER_GRABBED);
        hw.rightGrabberServo.setPosition(Hardware.GRABBER_GRABBED);

        // wait for the start button to be pressed.
        waitForStart();

        runtime.reset();

        // 1. Drive straight into wall
        // 2. Scan pictogram
        // 3. Hit jewel
        // 4. Put block we already have in cryptobox
        // ...

        try {
            /*while (false) {
                sleepSync();
                log("Waiting for touch");
            }*/
            // 3.25 sec

            //ADD moving back and forth if neither detected
            FutureTask<Void> jewelTask = new FutureTask<>(() -> {
                hw.jewelArmServo.setPosition(0);
                sleep(2000);
                if (hw.jewelColorSensor.blue() > 0) {
                    hw.jewelHandServo.setPosition(1);
                } else {
                    hw.jewelHandServo.setPosition(0);
                }
                sleep(250);
                hw.jewelHandServo.setPosition(0.5);
                hw.jewelArmServo.setPosition(0.5);
                sleep(1000);
                return null;
            });
            // ?? time
            FutureTask<RelicRecoveryVuMark> detectGlyphTask =
                    new FutureTask<>(this::detectPictogram);
            jewelTask.run();
            detectGlyphTask.run();

            // wait for jewel task to finish
            jewelTask.get();
            // Get (blocking) glyph column
            RelicRecoveryVuMark correctGlyphColumn;
            try {
                correctGlyphColumn = detectGlyphTask.get(10, TimeUnit.SECONDS);
            } catch (TimeoutException e) {
                correctGlyphColumn = RelicRecoveryVuMark.UNKNOWN;
            }
            log("Got glyph column " + correctGlyphColumn);

            // Turn 90 degrees to the left
            turn(hw, -90);
            hw.leftDriveMotor.setPower(-0.25);
            hw.rightDriveMotor.setPower(-0.25);
            sleep(1000);
            hw.leftDriveMotor.setPower(0);
            hw.rightDriveMotor.setPower(0);

            // Detect with ODS
            int requiredColumn;
            switch (correctGlyphColumn) {
                case LEFT:
                    requiredColumn = 3;
                    break;
                case RIGHT:
                    requiredColumn = 2;
                    break;
                case CENTER:
                    requiredColumn = 1;
                    break;
                case UNKNOWN:
                default:
                    requiredColumn = 2; // lmao
            }
            //hw.horizontalDriveMotor.setPower(0.25);
            int atColumn = 0;
            while (requiredColumn > atColumn) {
                while (hw.ods.getLightDetected() == 0) {
                    sleepSync();
                }
                // we have hit a glyph column wall
                atColumn += 1;
            }
            //hw.horizontalDriveMotor.setPower(0);

            // Now we are at the required column. Move forward until ODS reads.
            hw.leftDriveMotor.setPower(0.1);
            hw.rightDriveMotor.setPower(0.1);
            while (hw.ods.getLightDetected() == 0) {
                sleepSync();
            }
            hw.rightDriveMotor.setPower(0);
            hw.leftDriveMotor.setPower(0);
            // Hopefully we've hit the box by now. Release the box!
            hw.rightGrabberServo.setPosition(Hardware.GRABBER_RELEASED);
            hw.leftGrabberServo.setPosition(Hardware.GRABBER_RELEASED);

        } catch (Exception e) {
            log("Stopping op mode... " + e);
        }
        log("Stopped op mode");
    }
}
