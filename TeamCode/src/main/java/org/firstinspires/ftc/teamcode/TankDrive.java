package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Tank Drive")
public class TankDrive extends LinearOpMode {
    private Hardware hw;

    private void runDrivetrain() {
        hw.backLeftDrive.setPower(-gamepad1.left_stick_y);
        hw.frontLeftDrive.setPower(-gamepad1.left_stick_y);
        hw.backRightDrive.setPower(-gamepad1.right_stick_y);
        hw.frontRightDrive.setPower(-gamepad1.right_stick_y);
    }

    private void runArm() {
        double armPower = -gamepad2.right_stick_y;
        double extensionPower = -gamepad2.left_stick_y;

//        int newPosition = (int) (hw.leftArm.getCurrentPosition() + (50 * armPower));
//        if (Math.abs(armPower) > 0.1) {
//            hw.setArmPosition(newPosition);
//        }
//        hw.leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        hw.rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        hw.leftArm.setPower(armPower);
//        hw.rightArm.setPower(armPower);
        telemetry.addData("Arm position", hw.leftArm.getCurrentPosition());
        telemetry.addData("Arm position right", hw.rightArm.getCurrentPosition());

//        hw.armExtender.setPower(extensionPower);
//        telemetry.addData("Arm extender position", hw.armExtender.getCurrentPosition());
    }

    private void runGrabber() {
        double grabberPos = gamepad2.right_trigger;
        double grabberPivotPos = gamepad2.left_trigger;

        hw.grabberPivot.setPosition(grabberPivotPos);
        hw.grabber.setPosition(grabberPos);
        telemetry.addData("Grabber position", hw.grabber.getPosition());
        telemetry.addData("Grabber pivot pos", hw.grabberPivot.getPosition());

//        hw.armHolder.setPosition(gamepad2.a ? Hardware.ARM_HOLDER_RETRACTED : Hardware.ARM_HOLDER_HOLDING);
    }



    @Override
    public void runOpMode() throws InterruptedException {
        hw = new Hardware(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            runDrivetrain();
            runArm();
            runGrabber();

            hw.led.setPower(gamepad2.x ? 0.5 : 0);

            telemetry.update();
        }
    }
}
