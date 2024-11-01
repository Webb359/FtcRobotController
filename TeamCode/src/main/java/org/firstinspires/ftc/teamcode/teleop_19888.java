package org.firstinspires.ftc.teamcode;


    /* Copyright (c) 2017 FIRST. All rights reserved.
     *
     *
     */


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


//  DRIVING MECANUM WHEELS SIMPLIFIED
//             @TeleOp
//             public class MecanumTeleOp extends LinearOpMode {
//                 @Override
//                 public void runOpMode() throws InterruptedException {
//                     // Declare our motors
//                     // Make sure your ID's match your configuration
//                     // call hardware class here


//                     waitForStart();

//                     if (isStopRequested()) return;

//                     while (opModeIsActive()) {
//                         double y = -gamepad1.left_stick_y; // Remember, this is reversed!
//                         double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//                         double rx = gamepad1.right_stick_x;

//                         // Denominator is the largest motor power (absolute value) or 1
//                         // This ensures all the powers maintain the same ratio, but only when
//                         // at least one is out of the range [-1, 1]
//                         double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//                         double frontLeftPower = (y + x + rx) / denominator;
//                         double backLeftPower = (y - x + rx) / denominator;
//                         double frontRightPower = (y - x - rx) / denominator;
//                         double backRightPower = (y + x - rx) / denominator;

//                         motorFrontLeft.setPower(frontLeftPower);
//                         motorBackLeft.setPower(backLeftPower);
//                         motorFrontRight.setPower(frontRightPower);
//                         motorBackRight.setPower(backRightPower);
//                     }
//                 }
//             }
    /**
     * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
     * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
     * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
     * class is instantiated on the Robot Controller and executed.
     *
     */

    @TeleOp(name="TELEOP") // i will come soon. when the 3d print is done, make sure u watch ur back
//@Disabled
    public class teleop_19888 extends OpMode
    {

         RobotHardware robot = new RobotHardware();
        // Declare OpMode members.

        //methods to control the speed of the robot.
        float speedModifier = .65f;

        private float reductionModifier = .4f;//the amount that the speed will be decreased in precision mode. Should be < 1
        private float turboModifier = 1.9f;// the amount that the speed will be increased in turbo mode. Must be <2. No increase is 1.
        private float precisionActive = 1f;
        private float turnReduction = .5f;//reduces the speed of turning. <1 to reduce. 1 if to leave as normal> yuh
        //private float BRDrive = 1f;
        double stickX = 0;
        double stickY = 0;
        double stickR = 0;
        double vm = 0;
        @Override
        public void init() {
            //Initialize the hardware variables.
            //The init() method of the hardware class does all the work here
            robot.init(hardwareMap);



        }

        @Override
        public void loop() {
            mecanumMove();

        }

        public void mecanumMove() {

//        double clawDistanceMeasure = robot.clawDist.getDistance(DistanceUnit.MM);
//        telemetry.addData("Claw Distance", clawDistanceMeasure);
//        telemetry.addData("Distance from Back", robot.backDist.getDistance(DistanceUnit.CM));
//        telemetry.addData("Touch Sensor Pressed", robot.magStopBottom.getValue());//should indicate whether the touch sensor is pressed.


            //======================================
            //------------WHEEL CODE----------------
            //======================================


            if (Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y) >.2) {
                stickX = gamepad1.left_stick_x;
                stickY = gamepad1.left_stick_y;
            }
//            else {
//                stickX = gamepad2.left_stick_x;
//                stickY = gamepad2.left_stick_y;
//            }

            if (Math.abs(gamepad1.right_stick_x) >.2) {
                stickR = gamepad1.right_stick_x;
            }
//            else {
//                stickR = gamepad2.right_stick_x;
//            }
            //======================================
            //----------Lift--------------capyright 19888
            //======================================


            if (Math.abs(gamepad1.right_stick_y) >.2 ) {

                vm = gamepad1.right_stick_y;
            }
            if (Math.abs(gamepad2.right_stick_y) >.2 ) {

                vm = gamepad2.right_stick_y;
            }

            //int ticks = tickConversion * cmMove * direction;
        //  int tickConversion = (int)(Driving358.COUNTS_PER_MOTOR_REV/(3.14));
//            int cmMove = 0;
//            boolean enco=false;
//            //low level
//            if (gamepad2.a){
//                cmMove  = 15;
//                enco = true;
//            }
//            //mid level
//
//            if (gamepad2.b){
//                cmMove = 40;
//                enco = true;
//            }
//            //high level
//            if (gamepad2.y) {
//                cmMove = 55;
//                enco = true;
//            }
//            if (enco) {
//                int ticks = tickConversion * cmMove;/ robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.lift.setTargetPosition((ticks));
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.lift.setPower(.9);
//                while (robot.lift.isBusy()) {
//
//                }
//                robot.lift.setPower(0);
//                robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                enco = false;
//                cmMove = 0;
//            }
//
//            if (gamepad2.right_bumper){
//                robot.lift.setPower(-.5);
//                boolean test=true;
//                while (robot.lift.isBusy() || test ) {
//                    if(robot.touch.isPressed()){
//                        robot.lift.setPower(0);
//                        test=false;
//                        break;
//                    }
//
//                }



        //variables
        double r = Math.hypot(-stickX, stickY); //ur mom is watching you from the ceiling. dont look up...
        double robotAngle = Math.atan2(stickY, -stickX) - Math.PI / 4;
        double rightX = -stickR * turnReduction;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX; //the swedes are coming 4 u soon
            final double v4 = r * Math.cos(robotAngle) - rightX;

//            if (gamepad1.left_bumper || gamepad2.left_bumper) {//if the left bumper is pressed, it multiplies the total power by the precision driving modifer
//                precisionActive = reductionModifier;
//            } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
//                precisionActive = turboModifier;//right bumper = turbo mode (for crossing the barriers)
//            } else {
//                precisionActive = 1f; //no modifier
//            }
//            if (gamepad1.left_bumper) {//if the left bumper is pressed, it multiplies the total power by the precision driving modifer
//                precisionActive = reductionModifier;
//            } else if (gamepad1.right_bumper) {
//                precisionActive = turboModifier;//right bumper = turbo mode (for crossing the barriers)
//            } else {
//                precisionActive = 1f; //no modifier
//            }
            double powerLeft = -speedModifier * v1 * precisionActive;


            robot.leftFront.setPower(-speedModifier * v1 * precisionActive);
            robot.leftBack.setPower(-speedModifier * v2 * precisionActive);
            robot.rightFront.setPower(-speedModifier * v3 * precisionActive);
            robot.rightBack.setPower(-speedModifier * v4 * precisionActive);
          //robot.lift.setPower(-speedModifier * vm * precisionActive);

//            telemetry.addData("fLPower", -speedModifier * v1 * precisionActive);
//            telemetry.addData("fRPower", -speedModifier * v2 * precisionActive);
//            telemetry.addData("bLPower", -speedModifier * v3 * precisionActive);
//            telemetry.addData("bRPower", -speedModifier * v4 * precisionActive);
//
//            telemetry.addData("Encoder port 1 back left", robot.lb.getCurrentPosition());
//            telemetry.addData("Encoder port 2 front right", robot.rf.getCurrentPosition());
//            telemetry.addData("Encoder port 3 back right", robot.rb.getCurrentPosition());
//            telemetry.addData("Encoder port 4 back left", robot.lb.getCurrentPosition());
//
//            telemetry.addLine();

            //======================================
            //----------CLAW--------------
            //======================================


//                telemetry.addData("Right Trigger", gamepad2.x);
//            } else {
//                robot.leftServo.setPosition(0);
//                robot.rightServo.setPosition(1);
//                telemetry.addData("Neither", gamepad2.b);
                //======================================
                //----------CLAW ROTATOR----------------
                //======================================
            }
       }
//    }
//}
