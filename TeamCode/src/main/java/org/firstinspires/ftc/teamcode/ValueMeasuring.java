package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by 超高速 on 1/8/2017.
 */

@TeleOp(name = "Measure values of sensors", group = "Debugging")
public class ValueMeasuring extends OpMode {
    private Hardware robot = new Hardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        telemetry.addLine("Front ultrasonic distance: " +
                robot.sideFrontUltrasonicSensor.getUltrasonicLevel());
        telemetry.addLine("Back ultrasonic distance: " +
                robot.sideBackUltrasonicSensor.getUltrasonicLevel());
        telemetry.addLine("Side color sensor values: " + robot.sideColorSensor.argb());
        telemetry.addLine("Light sensor values: " + robot.floorLightSensor.getLightDetected());
    }
}
