package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.android.dex.util.ExceptionWithContext;

import java.util.concurrent.TimeUnit;

/**
 * Created by nitro on 10/9/2017.
 */

@Autonomous(name = "Autonomous program")
public class Experimental extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private String logText = "";
    private void log(String text) {
        String str = "[" + runtime.time(TimeUnit.SECONDS) + "] " + text;
        logText = str + logText;
        telemetry.addLine(logText);
        telemetry.update();
    }

    private void sleep() throws OpModeStoppedException {
        if (!opModeIsActive())
            throw new OpModeStoppedException();
        else
            idle();
    }

    @Override
    public void runOpMode() {
        Hardware hw = new Hardware(hardwareMap);

        // wait for the start button to be pressed.
        waitForStart();
        runtime.reset();

        try {
            hw.dcMotor.setPower(1);
            hw.servo.setPosition(1);
        } catch(Exception e) {
            System.out.println(e);
        }
    }
}
