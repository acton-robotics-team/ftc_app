package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 */
class Hardware
{
    public static final String ID_LEFT_DRIVE_MOTOR = "left_drive";
    private static final String ID_RIGHT_DRIVE_MOTOR = "right_drive";
    private static final String ID_LAUNCHER_MOTOR = "launcher";
    private static final String ID_SCOOPER_SERVO = "scooper";
    private static final String ID_BEACON_RIGHT_SERVO = "beacon_right_servo";
    private static final String ID_BEACON_LEFT_SERVO = "beacon_left_servo";
    private static final String ID_SIDE_COLOR_SENSOR = "side_color";
    private static final String ID_SIDE_FRONT_ULTRASONIC_SENSOR = "side_front_ultrasonic";
    private static final String ID_SIDE_BACK_ULTRASONIC_SENSOR = "side_back_ultrasonic";
    private static final String ID_FLOOR_LIGHT_SENSOR = "floor_light";
    private static final String ID_LAUNCHER_SERVO = "launcher_servo";

    public static final double POS_SCOOPER_SERVO_DOWN = 1;
    public static final double POS_SCOOPER_SERVO_UP = 1;
    public static final double POS_LAUNCHER_SERVO_DOWN = 1;
    public static final double POS_LAUNCHER_SERVO_UP = 1;
    public static final double POS_BEACON_SERVO_EXTENDED = 0.45;
    public static final double POS_BEACON_SERVO_RETRACTED = 0;

    /* Public OpMode members. */
    public DcMotor leftDriveMotor  = null;
    public DcMotor rightDriveMotor = null;
    public DcMotor launcherMotor = null;
    public Servo launcherServo = null;
    public Servo scooperServo = null;
    public Servo beaconRightServo = null;
    public Servo beaconLeftServo = null;
    public ColorSensor sideColorSensor = null;
    public UltrasonicSensor sideFrontUltrasonicSensor = null;
    public UltrasonicSensor sideBackUltrasonicSensor = null;
    public LightSensor floorLightSensor = null;

    /* local OpMode members. */
    private HardwareMap hwMap           =  null;
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
        launcherMotor   = hwMap.dcMotor.get(ID_LAUNCHER_MOTOR);
        leftDriveMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDriveMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftDriveMotor.setPower(0);
        rightDriveMotor.setPower(0);
        launcherMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        scooperServo = hwMap.servo.get(ID_SCOOPER_SERVO);
        scooperServo.setPosition(POS_SCOOPER_SERVO_DOWN);
        beaconRightServo = hwMap.servo.get(ID_BEACON_RIGHT_SERVO);
        beaconLeftServo = hwMap.servo.get(ID_BEACON_LEFT_SERVO);
        launcherServo = hwMap.servo.get(ID_LAUNCHER_SERVO);
        // Sensors
        sideColorSensor = hwMap.colorSensor.get(ID_SIDE_COLOR_SENSOR);
        sideFrontUltrasonicSensor = hwMap.ultrasonicSensor.get(ID_SIDE_FRONT_ULTRASONIC_SENSOR);
        sideBackUltrasonicSensor = hwMap.ultrasonicSensor.get(ID_SIDE_BACK_ULTRASONIC_SENSOR);
        floorLightSensor = hwMap.lightSensor.get(ID_FLOOR_LIGHT_SENSOR);
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