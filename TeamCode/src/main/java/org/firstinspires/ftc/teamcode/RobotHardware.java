package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.util.Range;

public class RobotHardware {
    private Telemetry telemetry;
    private LinearOpMode myOpMode = null;
    public IMU imu = null;
    public DcMotorEx leftFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightBack = null;
    public DcMotorEx leftArm = null;
    public DcMotorEx rightArm = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;

    private ElapsedTime runtime = new ElapsedTime();
    public static final double MID_SERVO = 0.5;

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: Rev HD Hex Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 0.05;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 6.0;    // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double ROBOT_WIDTH_INCHES = 11;       // Distance between left and right wheels

    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable.

    public RobotHardware(LinearOpMode opMode) {
        myOpMode = opMode;
        telemetry = opMode.telemetry;
    }

    public void init(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back");
        leftArm = hardwareMap.get(DcMotorEx.class, "left_arm");
        rightArm = hardwareMap.get(DcMotorEx.class, "right_arm");
//        leftClaw = hardwareMap.get(Servo.class, "left_hand");
//        rightClaw = hardwareMap.get(Servo.class, "right_hand");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

//        leftClaw.setPosition(MID_SERVO);
//        rightClaw.setPosition(MID_SERVO);


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
    public void rotate_gyro(double theta,double speed,double timeout){
        double heading=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

    }
    public void driveStraightEncoderGyro(double maxDriveSpeed,double y, double timeout){

        double heading=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double leftFrontDistance = (y * COUNTS_PER_INCH);
        double leftBackDistance = (y * COUNTS_PER_INCH);
        double rightFrontDistance = (y * COUNTS_PER_INCH);
        double rightBackDistance = (y * COUNTS_PER_INCH);

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
        double turnSpeed;
        double leftspeed;
        double rightspeed;
        while ((leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy())
                && runtime.seconds() < timeout
                && myOpMode.opModeIsActive()) {
            double currentheading=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double headingError;
            headingError = heading - currentheading;
            while (headingError > 180)  headingError -= 360;
            while (headingError <= -180) headingError += 360;
            turnSpeed=Range.clip(headingError * P_DRIVE_GAIN, -1, 1);

            if (y < 0) {
                turnSpeed *= -1.0;
            }
            leftspeed=maxDriveSpeed-turnSpeed;
            rightspeed=maxDriveSpeed+turnSpeed;
            double max = Math.max(Math.abs(leftspeed), Math.abs(rightspeed));
            if (max > maxDriveSpeed)
            {
                leftspeed /= max;
                rightspeed /= max;
            }

            leftFront.setPower(leftspeed);
            leftBack.setPower(leftspeed);
            rightFront.setPower(rightspeed);
            rightBack.setPower(rightspeed);
        }

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void drive(double x, double y, double theta, int gyro) {
        if (gyro==1){
            double currentheading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double headingError = -currentheading; // Correcting for gyro offset
            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;
            double turnSpeed = Range.clip(headingError * P_DRIVE_GAIN, -1, 1);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(theta + turnSpeed), 1);
            double leftFrontPower = (y + x + (theta + turnSpeed)) / denominator;
            double leftBackPower = (y - x + (theta + turnSpeed)) / denominator;
            double rightFrontPower = (y - x - (theta + turnSpeed)) / denominator;
            double rightBackPower = (y + x - (theta + turnSpeed)) / denominator;

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);

            telemetry.addData("Gyro Heading", currentheading);
            telemetry.addData("Heading Error", headingError);
            telemetry.addData("leftFrontPower", leftFrontPower);
            telemetry.addData("leftBackPower", leftBackPower);
            telemetry.addData("rightFrontPower", rightFrontPower);
            telemetry.addData("rightBackPower", rightBackPower);
        }
        else{
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

            double scaledSpeed = speed / 100.0;
            leftFront.setPower(scaledSpeed);
            leftBack.setPower(scaledSpeed);
            rightFront.setPower(scaledSpeed);
            rightBack.setPower(scaledSpeed);

            telemetry.addData("leftFrontPower", leftFront.getPower());
            telemetry.addData("leftBackPower", leftBack.getPower());
            telemetry.addData("rightFrontPower", rightFront.getPower());
            telemetry.addData("rightBackPower", rightBack.getPower());
            telemetry.addData("leftFrontPosition", leftFront.getCurrentPosition());
            telemetry.addData("leftBackPosition", leftBack.getCurrentPosition());
            telemetry.addData("rightFrontPosition", rightFront.getCurrentPosition());
            telemetry.addData("rightBackPosition", rightBack.getCurrentPosition());
            telemetry.update();
        }

        stopDrive();

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        setDrivePower(speed, speed, speed, speed);

        while ((leftFront.isBusy() || leftBack.isBusy() || rightFront.isBusy() || rightBack.isBusy())
                && runtime.seconds() < timeout
                && myOpMode.opModeIsActive()) {

            telemetry.addData("leftFrontPower", leftFront.getPower());
            telemetry.addData("leftBackPower", leftBack.getPower());
            telemetry.addData("rightFrontPower", rightFront.getPower());
            telemetry.addData("rightBackPower", rightBack.getPower());
            telemetry.addData("leftFrontPosition", leftFront.getCurrentPosition());
            telemetry.addData("leftBackPosition", leftBack.getCurrentPosition());
            telemetry.addData("rightFrontPosition", rightFront.getCurrentPosition());
            telemetry.addData("rightBackPosition", rightBack.getCurrentPosition());
            telemetry.update();
        }

        stopDrive();

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stopDrive() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void setDrivePower(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower) {
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    public void setArmPower(double power) {
        if (power == 0) {
            // When power is 0, set both arms to run to position mode
            int leftPos = leftArm.getCurrentPosition();
            int rightPos = rightArm.getCurrentPosition();
            // Use the average position as target
            int targetPos = (leftPos + rightPos) / 2;
            
            leftArm.setTargetPosition(targetPos);
            rightArm.setTargetPosition(targetPos);
            
            leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            // Set a small power to maintain position
            leftArm.setPower(0.1);
            rightArm.setPower(0.1);
        } else {
            // Normal velocity control for non-zero power
            double rpm = power * 100;
            double tickperrevl = leftArm.getMotorType().getTicksPerRev();
            double tickspersecl = (rpm/60) * tickperrevl;
            double tickperrevr = rightArm.getMotorType().getTicksPerRev();
            double tickspersecr = (rpm/60) * tickperrevr;
            
            leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            // Apply small corrections if arms get out of sync
            double positionDiff = leftArm.getCurrentPosition() - rightArm.getCurrentPosition();
            double correction = positionDiff * 0.01; // Small correction factor
            
            leftArm.setVelocity(tickspersecl - correction);
            rightArm.setVelocity(tickspersecr + correction);
        }
    }
    public void driveTimed(double x, double y, double theta, long timeMs) {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(theta), 1);
        double leftFrontPower = (y + x + theta) / denominator ;
        double leftBackPower = (y - x + theta) / denominator;
        double rightFrontPower = (y - x - theta) / denominator;
        double rightBackPower = (y + x - theta) / denominator;



        double numerator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(theta), 1);
        double leftFrontPowerActual = (y + x + theta) / numerator;
        double leftBackPowerActual = (y - x + theta) / numerator;
        double rightFrontPowerActual = (y - x - theta) / numerator;
        double rightBackPowerActual = (y + x - theta) / numerator;

        telemetry.addData("leftFrontPower", leftFrontPowerActual);
        telemetry.addData("leftBackPower", leftBackPowerActual);
        telemetry.addData("rightFrontPower", rightFrontPowerActual);
        telemetry.addData("rightBackPower", rightBackPowerActual);
        telemetry.addData("leftFrontPower", leftFront.getPower());
        telemetry.addData("leftBackPower", leftBack.getPower());
        telemetry.addData("rightFrontPower", rightFront.getPower());
        telemetry.addData("rightBackPower", rightBack.getPower());
        telemetry.update();

        runtime.reset();
        while (runtime.milliseconds() < timeMs && myOpMode.opModeIsActive()) {
            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);
            telemetry.addData("leftFrontPower", leftFront.getPower());
            telemetry.addData("leftBackPower", leftBack.getPower());
            telemetry.addData("rightFrontPower", rightFront.getPower());
            telemetry.addData("rightBackPower", rightBack.getPower());
            telemetry.addData("Time Remaining", timeMs - runtime.milliseconds());
            telemetry.update();
        }

        stopDrive();
    }
}
