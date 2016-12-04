package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Activate scooperServo servo", group = "Always-active")
public class ConceptActivateScooper extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo scooper = hardwareMap.servo.get(Hardware.ID_SCOOPER_SERVO);
        final int cycleMs = 50; // Help make sure that this is correct

        telemetry.addData(">", "Press start to activate scooper.");
        telemetry.update();

        waitForStart();

        scooper.setPosition(Hardware.POS_SCOOPER_SERVO_DOWN);
        boolean isUp = false;
        while (opModeIsActive()) {
            if (isUp) {
                scooper.setPosition(Hardware.POS_SCOOPER_SERVO_DOWN);
            } else {
                scooper.setPosition(Hardware.POS_SCOOPER_SERVO_UP);
            }

            sleep(cycleMs);
            idle();
        }
    }
}
