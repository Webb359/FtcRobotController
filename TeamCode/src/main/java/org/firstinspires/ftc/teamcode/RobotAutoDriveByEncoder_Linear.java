package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Robot: Auto Drive By Encoder Linear", group="Robot")
public class RobotAutoDriveByEncoder_Linear extends LinearOpMode {
    public RobotHardware robot = new RobotHardware(this);

    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData(">", "Robot Ready.  Press START.");
        telemetry.update();
        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();
        robot.drive_encoders(0, 100, 48, 5);

/*
    drive: forward
    pick up thing
    turn left
    forward
    score
    turn back
 */
    }
}
