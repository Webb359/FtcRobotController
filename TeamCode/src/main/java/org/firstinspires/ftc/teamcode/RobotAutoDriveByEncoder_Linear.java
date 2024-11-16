package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Drive By Time Linear", group="Robot")
public class RobotAutoDriveByTime_Linear extends LinearOpMode {
    public RobotHardware robot = new RobotHardware(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize the robot
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData(">", "Robot Ready. Press START.");
        telemetry.update();

        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();

        // Autonomous sequence

        // Step 1: Drive forward for 27 inches
        robot.driveTimed(0, 1, 0, 0.5, 3000); // Adjust time to match 27 inches

        // Step 2: Pick up object (move arm)
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1) {
            robot.setArmPower(0.5); // Move arm up for 1 second
        }
        robot.setArmPower(0);

        // Step 3: Turn left 90 degrees
        robot.driveTimed(0, 0, 1, 0.5, 1000); // Adjust time to match a 90-degree turn

        // Step 4: Drive forward for 105 inches
        robot.driveTimed(0, 1, 0, 0.5, 8000); // Adjust time to match 105 inches

        // Step 5: Turn left 90 degrees again
        robot.driveTimed(0, 0, 1, 0.5, 1000);

        // Step 6: Drive forward for 15.5 inches
        robot.driveTimed(0, 1, 0, 0.5, 1500); // Adjust time to match 15.5 inches

        // Step 7: Place object (move arm)
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1) {
            robot.setArmPower(-0.5); // Move arm down for 1 second
        }
        robot.setArmPower(0);

        // Step 8: Turn back to starting orientation
        robot.driveTimed(0, 0, -1, 0.5, 2000); // Adjust time to match a 180-degree turn

        // End of autonomous sequence
        robot.stopDrive();
        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
    }
}
