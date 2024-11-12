package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Drive By Encoder Linear", group="Robot")
public class RobotAutoDriveByEncoder_Linear extends LinearOpMode {
    public RobotHardware robot = new RobotHardware(this);
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData(">", "Robot Ready.  Press START.");
        telemetry.update();
        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();
        robot.drive_encoders(0, 27, 48, 5);
        // move arm to pick up thing
        robot.drive_encoders(0, 105, 48, 5);
        robot.rotate_encoders(90, 48, 5);
        // move arm to score
        robot.drive_encoders(0, 15.5, 48, 5);
        robot.rotate_encoders(-90, 48, 5);
        // move arm to place thing
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < 1) {
            robot.setArmPower(50);
        }
        while (opModeIsActive() && runtime.seconds() < 2) {
            robot.setArmPower(-30);
        }
        robot.setArmPower(0);
        robot.drive_encoders(0, -10, 48, 5);
        robot.stopDrive();
/*
    drive: (inches)
    forward: 27
    pick up thing
    turn left
    forward: 105
    turn left
    forward: 15.5
    score
    turn back
*/
    }
}
