package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Robot: Teleop POV", group="Robot")
public class RobotTeleopPOV_Linear extends LinearOpMode {
    public RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        telemetry.addData(">", "Robot Ready.  Press START.");
        telemetry.update();
        waitForStart();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        boolean gamepad1A_ispressed = false;
        while (opModeIsActive()) {


            if (gamepad1.right_trigger>0.1){
                robot.setArmPower(gamepad1.right_trigger/3);
            }
            else if (gamepad1.right_bumper){
                robot.setArmPower(-0.2);
            }
            else{
                robot.setArmPower(0.05);
            }

            if (gamepad1.a){
                if (!gamepad1A_ispressed){
                    robot.moveClaw();
                    gamepad1A_ispressed = true;
                }
            }
            else if (gamepad1A_ispressed){
                gamepad1A_ispressed = false;
            }

            if(gamepad1.dpad_up&&robot.rightExtend.getPosition()<0.99){
                robot.setExtend(robot.rightExtend.getPosition()+0.01);
            }
            else if(gamepad1.dpad_down&&robot.rightExtend.getPosition()>0.01){
                robot.setExtend(robot.rightExtend.getPosition()-0.01);
            }

            if (gamepad1.dpad_right&&robot.rightHand.getPosition()<0.64){
                robot.setHand(robot.rightHand.getPosition()+0.004);
            }
            else if(gamepad1.dpad_left&&robot.rightHand.getPosition()>0.46){
                robot.setHand(robot.rightHand.getPosition()-0.004);
            }

            double drive = -gamepad1.left_stick_y * 0.3;
            double strafe = gamepad1.left_stick_x * 0.3;
            double rotate = gamepad1.right_stick_x * 0.3;
            robot.drive(strafe, drive, rotate,0);
            telemetry.addData("leftFrontPosition", robot.leftFront.getCurrentPosition());
            telemetry.addData("leftBackPosition", robot.leftBack.getCurrentPosition());
            telemetry.addData("rightFrontPosition", robot.rightFront.getCurrentPosition());
            telemetry.addData("rightBackPosition", robot.rightBack.getCurrentPosition());
            telemetry.addData("Status", "Running");
            telemetry.addData("imu", robot.imu.getRobotYawPitchRollAngles().toString());
            telemetry.addData("claw state",robot.clawOpen);
            telemetry.update();
        }
    }
}
