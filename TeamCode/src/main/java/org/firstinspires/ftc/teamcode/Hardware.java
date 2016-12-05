package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 */
public class Hardware
{
    public static final String ID_LEFT_DRIVE_MOTOR = "left_drive";
    public static final String ID_RIGHT_DRIVE_MOTOR = "right_drive";
    public static final String ID_SWEEPER_MOTOR = "sweeper";
    public static final String ID_LAUNCHER_MOTOR = "launcher";
    public static final String ID_SCOOPER_SERVO = "scooper";

    public static final double POS_SCOOPER_SERVO_DOWN = 1;
    public static final double POS_SCOOPER_SERVO_UP =1;

    /* Public OpMode members. */
    public DcMotor leftDriveMotor  = null;
    public DcMotor rightDriveMotor = null;
    public DcMotor sweeperMotor = null;
    public DcMotor launcherMotor = null;
    public Servo scooperServo = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware() { }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDriveMotor  = hwMap.dcMotor.get(ID_LEFT_DRIVE_MOTOR);
        rightDriveMotor = hwMap.dcMotor.get(ID_RIGHT_DRIVE_MOTOR);
        sweeperMotor    = hwMap.dcMotor.get(ID_SWEEPER_MOTOR);
        launcherMotor   = hwMap.dcMotor.get(ID_LAUNCHER_MOTOR);
        leftDriveMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDriveMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftDriveMotor.setPower(0);
        rightDriveMotor.setPower(0);
        sweeperMotor.setPower(0);
        launcherMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        scooperServo = hwMap.servo.get(ID_SCOOPER_SERVO);
        scooperServo.setPosition(POS_SCOOPER_SERVO_DOWN);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}