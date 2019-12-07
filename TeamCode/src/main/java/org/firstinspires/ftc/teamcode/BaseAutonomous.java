package org.firstinspires.ftc.teamcode;

import android.content.ContentResolver;
import android.content.res.AssetManager;
import android.os.Environment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryConfig;
import com.acmerobotics.roadrunner.trajectory.TrajectoryLoader;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.tank.SuperiorestTankDrive;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.List;

@Autonomous
public class BaseAutonomous extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

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

    private SuperiorestTankDrive drive;


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

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void driveDistance(double in) {
        double initialDistanceIn = drive.getWheelPositions().get(0);
        drive.setMotorPowers(0.5, 0.5);
        while (opModeIsActive() && drive.getWheelPositions().get(0) < initialDistanceIn + in) {}
        drive.setMotorPowers(0, 0);
    }

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();
        tfod.activate();
        drive = new SuperiorestTankDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(12, 24 + 24 + 12, Math.toRadians(270)));

        File file = new File(Environment.getExternalStorageDirectory() + "/" + File.separator + "BlueAutonomous.yaml");
        InputStream stream;
        try {
            file.createNewFile();
            stream = hardwareMap.appContext.getAssets().open("trajectory/BlueAutonomous.yaml");
            // write the bytes in file
            if (file.exists()) {
                byte[] buffer = new byte[stream.available()];
                stream.read(buffer);

                OutputStream outStream = new FileOutputStream(file);
                outStream.write(buffer);
                outStream.close();
                System.out.println("file created: " + file);
            }
        } catch (IOException e) {
            throw new IllegalArgumentException("Failure to create file.");
        }

        Trajectory traj = TrajectoryLoader.load(
                new File(Environment.getExternalStorageDirectory().getPath() + "/BlueAutonomous.yaml"));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        drive.followTrajectorySync(traj);
        drive.setMotorPowers(0.2, 0.2);

        // Now we are in front of the skystones; scan for skystones
        while (opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        float recognitionMiddle = (recognition.getLeft() + recognition.getRight()) / 2;

                        int midpoint = recognition.getImageWidth() / 2;
                        telemetry.addData("Recognition middle", recognitionMiddle);
                        telemetry.addData("Midpoint", midpoint);

                        if (recognitionMiddle > midpoint) {
                            telemetry.addLine("Reached center.");
                            break;
                        }
                    }
                    telemetry.update();
                }
            }
        }

        drive.turnSync(Math.toRadians(-90));
        driveDistance(6);

        if (tfod != null) {
            tfod.shutdown();
        }
    }
}
