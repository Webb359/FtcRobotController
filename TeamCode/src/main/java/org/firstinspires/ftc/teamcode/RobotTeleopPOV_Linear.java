package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Robot: Teleop POV", group="Robot")
public class RobotTeleopPOV_Linear extends LinearOpMode {
    public RobotHardware robot = new RobotHardware(this);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        telemetry.addData(">", "Robot Ready.  Press START.");    //
        telemetry.update();
        waitForStart();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                robot.setArmPower(0.5);
            }
            else if (gamepad1.right_trigger > 0.5) {
                robot.setArmPower(-0.5);
            }
            else{
                robot.setArmPower(0);
            }
            robot.drive(gamepad1.left_stick_x, gamepad1.left_stick_y*0.5, gamepad1.right_stick_x);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }


    }
}
