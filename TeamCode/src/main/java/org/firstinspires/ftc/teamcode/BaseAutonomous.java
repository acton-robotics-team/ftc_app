package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveREVOptimized;

import java.util.List;

@SuppressWarnings("StatementWithEmptyBody")
public abstract class BaseAutonomous extends LinearOpMode {
    public enum Alliance {
        RED, BLUE
    }

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_STONE = "Stone";
    private static final String LABEL_SKYSTONE = "Skystone";

    private static final int VIEWPORT_WIDTH = 1280;
    private static final int VIEWPORT_HEIGHT = 720;

    private Alliance alliance;

    public BaseAutonomous(Alliance alliance) {
        this.alliance = alliance;
    }

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "Adh8C5H/////AAABmVDzpiKqBUSMoQYx5/Y4ngk6LPlWrp/xswdYG+NsdkrytPZYUWbgTrn12RndiSHmeE7U4SahXVXh+LzHk6A2NcDJtw7PRISyVL2LkQKLmVAku15aVDWwrsS95SYBw6CU3v1oDJS8E04D/e81gV+OgbL2fr0kK8+vfYsVMjQ07H3LcPwwl3Xc7z62wqh/sBZZIS6go83mUb28wPspABn0ly19ez566ZlZEtgYxUOJeqQ2wAJsIbTEDYjb7gt7lectCCLZY3Gziuyv21WntozJjdHgNAde2X+E2hxcE6TP85n6U2+dR3RddREehvk2C5swsmB5pvgmtB4xoOgL3IIWjqGrhrJ7n00n+RdYbObKeItT";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private Hardware hw;
    private MecanumDriveREVOptimized drive;


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.5;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE, LABEL_SKYSTONE);
    }

    @SuppressLint("DefaultLocale")
    private SkystoneLocation detectSkystone() {
        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        List<Recognition> recognitions = tfod.getRecognitions();
        telemetry.addData("# Object Detected", recognitions.size());

        // step through the list of recognitions and display boundary info.
        int i = 0;
        for (Recognition recognition : recognitions) {
            telemetry.addData(String.format("label (%d)", i),
                    recognition.getLabel());
            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                    recognition.getLeft(), recognition.getTop());
            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                    recognition.getRight(), recognition.getBottom());

            float recognitionMiddle = (recognition.getLeft() + recognition.getRight()) / 2;

            if (recognition.getLabel().equals(LABEL_SKYSTONE)) {
                if (recognitionMiddle < VIEWPORT_WIDTH / 3.) {
                    return SkystoneLocation.LEFT;
                } else if (recognitionMiddle < VIEWPORT_WIDTH * (2. / 3)) {
                    return SkystoneLocation.MIDDLE;
                } else {
                    return SkystoneLocation.RIGHT;
                }
            }
        }
        return null;
    }

    private Pose2d mirror(int redX, int redY, double redHeading) {
        if (alliance == Alliance.RED) {
            return new Pose2d(redX, redY, redHeading);
        } else {
            return new Pose2d(redX, -redY, redHeading + Math.PI);
        }
    }

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();
        tfod.activate();
        hw = new Hardware(hardwareMap);
        drive = new MecanumDriveREVOptimized(hardwareMap);
        drive.setPoseEstimate(mirror(-36, -66, Math.toRadians(90)));

        SkystoneLocation stoneLocation = null;
        while (!opModeIsActive() && !isStopRequested()) {
            SkystoneLocation newLocation = detectSkystone();
            if (newLocation != null) {
                stoneLocation = newLocation;
            }

            telemetry.addData("Status", "Initialized");
            telemetry.addData("Skystone location", stoneLocation);
            telemetry.update();
        }

        if (stoneLocation == null) {
            // Whatever, just go for the left
            stoneLocation = SkystoneLocation.LEFT;
        }

        hw.arm.setPower(0.2);
        hw.arm.setTargetPosition(Hardware.ARM_LIFT_2);

        switch (stoneLocation) {
            case RIGHT:
                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .splineTo(mirror(-28, -40, Math.toRadians(90))).build());
                break;
            case MIDDLE:
                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .splineTo(mirror(-36, -40, Math.toRadians(90))).build());
                break;
            case LEFT:
                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .splineTo(mirror(-44, -40, Math.toRadians(90))).build());
                break;
        }
        hw.arm.setTargetPosition(Hardware.ARM_GRAB);
        hw.rightClaw.setPosition(Hardware.CLAW_OPEN);
        hw.leftClaw.setPosition(Hardware.CLAW_OPEN);
        while (opModeIsActive() && hw.arm.isBusy()) ;

        hw.rightClaw.setPosition(Hardware.CLAW_CLOSED);
        hw.leftClaw.setPosition(Hardware.CLAW_CLOSED);
        sleep(500);

        hw.arm.setTargetPosition(Hardware.ARM_LIFT_2);
        while (opModeIsActive() && hw.arm.isBusy()) ;

        // Travel to foundation
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(mirror(-36, -50, Math.toRadians(90)))
                .splineTo(mirror(48, -30, Math.toRadians(90))).build());

        // Release block
        hw.arm.setTargetPosition(Hardware.ARM_LIFT_1);
        hw.leftClaw.setPosition(Hardware.CLAW_OPEN);
        hw.rightClaw.setPosition(Hardware.CLAW_OPEN);
        sleep(500);
//
//        // Go back to get second block
//        hw.arm.setTargetPosition(Hardware.ARM_LIFT_2);
//        switch (stoneLocation) {
//            case RIGHT:
//                drive.followTrajectorySync(drive.trajectoryBuilder()
//                        .splineTo(mirror(-52, -30, Math.toRadians(90))).build());
//                break;
//            case MIDDLE:
//                drive.followTrajectorySync(drive.trajectoryBuilder()
//                        .splineTo(mirror(-60, -30, Math.toRadians(90))).build());
//                break;
//            case LEFT:
//                drive.followTrajectorySync(drive.trajectoryBuilder()
//                        .splineTo(mirror(-68, -30, Math.toRadians(90))).build());
//                break;
//        }
//        hw.arm.setTargetPosition(Hardware.ARM_GRAB);
//        while (opModeIsActive() && hw.arm.isBusy()) ;
//
//        hw.rightClaw.setPosition(Hardware.CLAW_CLOSED);
//        hw.leftClaw.setPosition(Hardware.CLAW_CLOSED);
//        sleep(500);
//
//        // Go to foundation and drop
//        drive.followTrajectorySync(drive.trajectoryBuilder()
//                .splineTo(mirror(48, -30, Math.toRadians(90))).build());
//
//        // Release block
//        hw.arm.setTargetPosition(Hardware.ARM_LIFT_1);
//        hw.leftClaw.setPosition(Hardware.CLAW_OPEN);
//        hw.rightClaw.setPosition(Hardware.CLAW_OPEN);
//
//        // Drag foundation
        drive.turnSync(Math.toRadians(-90));
        hw.leftFoundation.setPosition(1);
        hw.rightFoundation.setPosition(1);
        sleep(500);

        // Back up and place foundation
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(mirror(40, -48, Math.toRadians(0))).build());

        // Park under skybridge
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(mirror(0, -48, Math.toRadians(180))).build());

        if (tfod != null) {
            tfod.shutdown();
        }
    }
}
