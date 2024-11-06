package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive, and arm) and two servos (left_hand and right_hand).
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 * It is recommended to make the actual hardware objects private so that they can't be accessed externally.
 */

public class RobotHardware {

    /* Declare OpMode members. */
    // private LinearOpMode myOpMode = null;   // Gain access to methods in the calling OpMode.

    // Define Motor and Servo objects (Make them private so they can't be accessed externally)
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public DcMotor rightFront = null;
    public DcMotor leftArm = null;
    public DcMotor rightArm = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;
    public DcMotor elevatorLeft = null;
    public DcMotor elevatorRight = null;

    // Define Drive constants. Make them public so they CAN be used by the calling OpMode
    public static final double MID_SERVO = 0.5;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    // public RobotHardware(LinearOpMode opmode) { myOpMode = opmode; }

    public RobotHardware() {
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map and initialized.
     */
    public void init(HardwareMap hardwareMap) {
        // Define and Initialize Motors
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        leftArm = hardwareMap.get(DcMotor.class, "left_arm");
        rightArm = hardwareMap.get(DcMotor.class, "right_arm");
        leftClaw = hardwareMap.get(Servo.class, "left_hand"); // Corrected to Servo.class
        rightClaw = hardwareMap.get(Servo.class, "right_hand"); // Corrected to Servo.class
        elevatorLeft = hardwareMap.get(DcMotor.class, "elevator_left");
        elevatorRight = hardwareMap.get(DcMotor.class, "elevator_right");

        // To drive forward, most robots need the motor on one side to be reversed.
        // Adjust these directions based on your first test drive.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        // Initialize servos
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);

        // Set motor directions for arms
        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.FORWARD);

        // Optional: If encoders are connected, set them to run using encoders for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Optional: Telemetry code can be added here if you want to track hardware status
        // telemetry.addData(">", "Hardware Initialized");
        // myOpMode.telemetry.update();
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param Drive Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param Turn  Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveRobot(double Drive, double Turn) {
        // Combine drive and turn for blended motion.
        double left = Drive + Turn;
        double right = Drive - Turn;

        // Scale the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        // Use existing function to drive both wheels.
        setDrivePower(left, right);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftWheel  Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightWheel Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftWheel, double rightWheel) {
        // Output the values to the motor drives.
        leftFront.setPower(leftWheel);
        rightFront.setPower(rightWheel);
        leftBack.setPower(leftWheel);
        rightBack.setPower(rightWheel);
    }

    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset The offset to apply to the servo positions
     */
    public void setHandPositions(double offset) {
        offset = Range.clip(offset, -0.5, 0.5); // Clamp the offset to prevent out-of-bounds values
        leftClaw.setPosition(MID_SERVO + offset);
        rightClaw.setPosition(MID_SERVO - offset);
    }
}
