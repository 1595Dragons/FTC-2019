package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoTestBasic", group = "Test")
public class AutoTestBasic extends LinearOpMode{
    private static final double DRIVE_SPEED = .2, TURN_SPEED = 1, ARM_SPEED = .8, SIDE_SPEED = .25;

    // Config for the robot
    private Config robot = new Config(this);

    @Override
    public void runOpMode(){
        // Setup robot hardware
        robot.ConfigureRobtHardware();
        // Send telemetry message to signify robot waiting;
        robot.status("Resetting motors");
        robot.resetMotorsForAutonomous(robot.left_back, robot.left_front, robot.right_back, robot.right_front);

        waitForStart();

        robot.distinctDrive(DRIVE_SPEED,10,10,10,10,3);


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


}
