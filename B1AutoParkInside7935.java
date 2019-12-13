package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "B1AutoParkInside7935", group = "Official")
//@Disabled
public class B1AutoParkInside7935 extends LinearOpMode{


    //basic set up  code for search: 1001
    private static final double DRIVE_SPEED = .5, TURN_SPEED = .5, ARM_SPEED = .8, SIDE_SPEED = .5;

    private Config7935 robot = new Config7935(this);

    public void runOpMode() {

        //basic set up code1001
        robot.ConfigureRobtHardware();
        robot.ConfigureVision();
        robot.resetMotorsForAutonomous(robot.left_back, robot.left_front, robot.right_back, robot.right_front);

        robot.status("ready");
        waitForStart();
        robot.DriveForward(DRIVE_SPEED,25,3);
        robot.TurnByImu(TURN_SPEED,90,2);
        robot.DriveForward(DRIVE_SPEED,30,2);


    }
}
