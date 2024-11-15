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

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                robot.setArmPower(1.5);
            } else if (gamepad1.right_trigger > 0.3) {
                robot.setArmPower(1.5);
            } else {
                robot.setArmPower(0);
            }

            double drive = -gamepad1.left_stick_y * 0.5;
            double strafe = gamepad1.left_stick_x * 0.5;
            double rotate = gamepad1.right_stick_x * 0.5;
            robot.drive(strafe, drive, rotate);

            telemetry.addData("Arm Right Encoder Pos", robot.rightArm.getCurrentPosition());
            telemetry.addData("Arm Left Encoder Pos", robot.leftArm.getCurrentPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
