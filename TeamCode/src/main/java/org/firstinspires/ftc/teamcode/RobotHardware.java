package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
    public Servo Claw = null;
    public DcMotorEx arm=null;
    public Servo rightExtend;
    public Servo leftExtend;
    public Servo rightHand;
    public Servo leftHand;

    private ElapsedTime runtime = new ElapsedTime();
    public static final double MID_SERVO = 0.5;

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: Rev HD Hex Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.0;    // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double ROBOT_WIDTH_INCHES = 11;       // Distance between left and right wheels

    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable.

    double targetHeading = 0;

    public boolean clawOpen;

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

        Claw = hardwareMap.get(Servo.class, "claw");
        arm=hardwareMap.get(DcMotorEx.class, "arm");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightExtend = hardwareMap.get(Servo.class, "rightExtend");
        leftExtend = hardwareMap.get(Servo.class, "leftExtend");
        rightHand = hardwareMap.get(Servo.class, "rightHand");
        leftHand = hardwareMap.get(Servo.class, "leftHand");

        Claw.setPosition(0.55);
        clawOpen=true;

        rightExtend.setPosition(0.3);
        leftExtend.setPosition(0.7);
        rightHand.setPosition(0.5);
        leftHand.setPosition(0.5);
        // Set the IMU parameters for the robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    public void rotate_gyro(double theta,double speed,double timeout){
        double heading=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double targetHeading = heading + theta;
        
        // Normalize target heading to be within -180 to 180 degrees
        while (targetHeading > 180) targetHeading -= 360;
        while (targetHeading <= -180) targetHeading += 360;
        
        runtime.reset();
        
        // Set motors to run using encoders for better speed control
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        while (runtime.seconds() < timeout && myOpMode.opModeIsActive()) {
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double headingError = targetHeading - currentHeading;
            
            // Normalize heading error to be within -180 to 180 degrees
            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;
            
            // Calculate turn power using proportional control
            double turnSpeed = Range.clip(headingError * P_TURN_GAIN, -speed, speed);
            
            // Stop if we're close enough to target heading
            if (Math.abs(headingError) < 2.0) {
                break;
            }
            
            // Apply motor powers
            leftFront.setPower(turnSpeed);
            leftBack.setPower(turnSpeed);
            rightFront.setPower(-turnSpeed);
            rightBack.setPower(-turnSpeed);
            
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Heading Error", headingError);
            telemetry.addData("Turn Speed", turnSpeed);
            telemetry.update();
        }
        
        // Stop all motors
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
    public void driveStraightEncoderGyro(double maxDriveSpeed, double y, double timeout){

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

            telemetry.addData("left speed",leftspeed);
            telemetry.addData("right speed",rightspeed);
            telemetry.addData("leftFrontTarget", leftFront.getTargetPosition());
            telemetry.addData("leftBackTarget", leftBack.getTargetPosition());
            telemetry.addData("rightFrontTarget", rightFront.getTargetPosition());
            telemetry.addData("rightBackTarget", rightBack.getTargetPosition());
            telemetry.addData("leftFrontCurrent", leftFront.getCurrentPosition());
            telemetry.addData("leftBackCurrent", leftBack.getCurrentPosition());
            telemetry.addData("rightFrontCurrent", rightFront.getCurrentPosition());
            telemetry.addData("rightBackCurrent", rightBack.getCurrentPosition());
            telemetry.update();

        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
    public void drive_gyro(double x, double y, double theta){
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double leftFrontPower=0;
        double leftBackPower=0;
        double rightFrontPower=0;
        double rightBackPower=0;
        double currentheading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double headingError=0;
        if (theta==0) {
            headingError = currentheading - targetHeading; // Correcting for gyro offset
            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;
            double turnSpeed = Range.clip(headingError * P_DRIVE_GAIN, -1, 1);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(theta + turnSpeed), 1);
            leftFrontPower = (y + x + (turnSpeed)) / denominator;
            leftBackPower = (y - x + (turnSpeed)) / denominator;
            rightFrontPower = (y - x - (turnSpeed)) / denominator;
            rightBackPower = (y + x - (turnSpeed)) / denominator;
            telemetry.addData("gyro status","on");
        }
        else{
            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
            leftFrontPower = (y + x + theta) / denominator;
            leftBackPower = (y - x + theta) / denominator;
            rightFrontPower = (y - x - theta) / denominator;
            rightBackPower = (y + x- theta) / denominator;
            targetHeading=currentheading;
        }
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);

        telemetry.addData("Gyro Heading", currentheading);
        telemetry.addData("Target Heading", targetHeading);
        telemetry.addData("Heading Error", headingError);
        telemetry.addData("leftFrontPower", leftFrontPower);
        telemetry.addData("leftBackPower", leftBackPower);
        telemetry.addData("rightFrontPower", rightFrontPower);
        telemetry.addData("rightBackPower", rightBackPower);
    }

    public void drive_power(double x, double y, double theta){
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
    public void drive(double x, double y, double theta, int gyro) {
        if (gyro==1){
            drive_gyro(x,y,theta);
        }
        else{
            drive_power(x,y,theta);
        }
       
    }

    public void setArmPower(double p){
        arm.setPower(p);
    }

    public void setExtend(double p){
        rightExtend.setPosition(p);
        leftExtend.setPosition(1-p);
    }

    public void setHand(double p){
        rightHand.setPosition(p);
        leftHand.setPosition(1-p);
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

    public void moveClaw(){
        if (clawOpen){
            Claw.setPosition(0.44);
            clawOpen=false;
        }
        else{
            Claw.setPosition(0.55);
            clawOpen=true;
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
