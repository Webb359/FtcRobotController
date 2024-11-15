package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware {
    private Telemetry telemetry;
    private LinearOpMode myOpMode = null;
    public IMU imu = null;
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor leftArm = null;
    public DcMotor rightArm = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;

    private ElapsedTime runtime = new ElapsedTime();
    public static final double MID_SERVO = 0.5;

    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: Rev HD Hex Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;    // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double ROBOT_WIDTH_INCHES = 11;       // Distance between left and right wheels

    public RobotHardware(LinearOpMode opMode) {
        myOpMode = opMode;
        telemetry = opMode.telemetry;
    }

    public void init(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        leftArm = hardwareMap.get(DcMotor.class, "left_arm");
        rightArm = hardwareMap.get(DcMotor.class, "right_arm");
//        leftClaw = hardwareMap.get(Servo.class, "left_hand");
//        rightClaw = hardwareMap.get(Servo.class, "right_hand");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

//        leftClaw.setPosition(MID_SERVO);
//        rightClaw.setPosition(MID_SERVO);


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the encoders to reverse the direction of the arm motors
        leftArm.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(DcMotor.Direction.REVERSE);

        // Set the encoders for arm to zero
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the encoders for arm to on
        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the IMU parameters for the robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
    }

    public void drive(double x, double y, double theta) {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(theta), 1);
        double leftFrontPower = (y + x + theta) / denominator;
        double leftBackPower = (y - x + theta) / denominator;
        double rightFrontPower = (y - x - theta) / denominator;
        double rightBackPower = (y + x - theta) / denominator;

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);

        telemetry.addData("leftFrontPower", leftFrontPower);
        telemetry.addData("leftBackPower", leftBackPower);
        telemetry.addData("rightFrontPower", rightFrontPower);
        telemetry.addData("rightBackPower", rightBackPower);
        
    }

    public void drive_encoders(double x, double y, double speed, double timeout) {
        // Amplification factor for lateral movement due to mecanum wheel rollers (45-degree angle)
        final double LATERAL_AMPLIFICATION_FACTOR = 1.4;

        // Calculate distances accounting for forward (y) and amplified lateral (x) motion
        double leftFrontDistance = (y * COUNTS_PER_INCH) + (x * COUNTS_PER_INCH * LATERAL_AMPLIFICATION_FACTOR);
        double leftBackDistance = (y * COUNTS_PER_INCH) - (x * COUNTS_PER_INCH * LATERAL_AMPLIFICATION_FACTOR);
        double rightFrontDistance = (y * COUNTS_PER_INCH) - (x * COUNTS_PER_INCH * LATERAL_AMPLIFICATION_FACTOR);
        double rightBackDistance = (y * COUNTS_PER_INCH) + (x * COUNTS_PER_INCH * LATERAL_AMPLIFICATION_FACTOR);

        // Set target positions for each motor
        leftFront.setTargetPosition((int) leftFrontDistance + leftFront.getCurrentPosition());
        leftBack.setTargetPosition((int) leftBackDistance + leftBack.getCurrentPosition());
        rightFront.setTargetPosition((int) rightFrontDistance + rightFront.getCurrentPosition());
        rightBack.setTargetPosition((int) rightBackDistance + rightBack.getCurrentPosition());

        telemetry.addData("leftFrontTarget", leftFront.getTargetPosition());
        telemetry.addData("leftBackTarget", leftBack.getTargetPosition());
        telemetry.addData("rightFrontTarget", rightFront.getTargetPosition());
        telemetry.addData("rightBackTarget", rightBack.getTargetPosition());

        // Set motors to run to target position
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        // Run the motors until they reach the target or timeout occurs
        while ((leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy())
                && runtime.seconds() < timeout
                && myOpMode.opModeIsActive()) {

            leftFront.setPower(speed/100);
            leftBack.setPower(speed/100);
            rightFront.setPower(speed/100);
            rightBack.setPower(speed/100);

            telemetry.addData("leftFrontPower", leftFront.getPower());
            telemetry.addData("leftBackPower", leftBack.getPower());
            telemetry.addData("rightFrontPower", rightFront.getPower());
            telemetry.addData("rightBackPower", rightBack.getPower());
            telemetry.addData("leftFrontPosition", leftFront.getCurrentPosition());
            telemetry.addData("leftBackPosition", leftBack.getCurrentPosition());
            telemetry.addData("rightFrontPosition", rightFront.getCurrentPosition());
            telemetry.addData("rightBackPosition", rightBack.getCurrentPosition());
        }

        stopDrive();

        // Set motors to run without encoder after finishing the move
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("leftFrontPosition", leftFront.getCurrentPosition());
        telemetry.addData("leftBackPosition", leftBack.getCurrentPosition());
        telemetry.addData("rightFrontPosition", rightFront.getCurrentPosition());
        telemetry.addData("rightBackPosition", rightBack.getCurrentPosition());
    }

    public void rotate_encoders(double theta, double speed, double timeout) {
        double radians = Math.toRadians(theta);
        double arcLength = (ROBOT_WIDTH_INCHES / 2.0) * radians;

        // Convert arc length to encoder counts
        int targetPositionCounts = (int) (arcLength * COUNTS_PER_INCH);

        // Set target positions for rotating the robot base
        // Left side moves forward and right side moves backward for a counter-clockwise rotation
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + targetPositionCounts);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + targetPositionCounts);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() - targetPositionCounts);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() - targetPositionCounts);

        // Set motors to run to position mode
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        // Set motor power to initiate the rotation
        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);

        // Loop until rotation completes or timeout is reached
        while ((leftFront.isBusy() || leftBack.isBusy() || rightFront.isBusy() || rightBack.isBusy())
                && runtime.seconds() < timeout
                && myOpMode.opModeIsActive()) {

        telemetry.addData("leftFrontPosition", leftFront.getCurrentPosition());
            telemetry.addData("leftBackPosition", leftBack.getCurrentPosition());
            telemetry.addData("rightFrontPosition", rightFront.getCurrentPosition());
            telemetry.addData("rightBackPosition", rightBack.getCurrentPosition());

        }

        // Stop the motors after rotation completes
        stopDrive();

        // Reset motors to run without encoder mode for normal operation
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void setArmPower(double power) {
        leftArm.setPower(power);
        rightArm.setPower(power);
        telemetry.addData("Arm Power",power);
    }

    public void setClawPosition(double position) {
        leftClaw.setPosition(position);
        rightClaw.setPosition(position);
    }

    public void setDrivePower(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower) {
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    public void stopDrive() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void stopArm() {
        leftArm.setPower(0);
        rightArm.setPower(0);
    }

    public void stopClaw() {
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);
    }

    public void stopAll() {
        stopDrive();
        stopArm();
        stopClaw();
    }
}
