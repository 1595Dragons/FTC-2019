package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "2020ParkBlueFoundationOut", group = "Official")
//@Disabled
public class BlueParkFoundationOut2020 extends LinearOpMode {


    //basic set up  code for search: 1001
    private static final double DRIVE_SPEED = .5, TURN_SPEED = .5, ARM_SPEED = .8, SIDE_SPEED = .5;

    private Config6128 robot = new Config6128(this);

    public void runOpMode() {

        //basic set up code1001
        robot.ConfigureRobtHardware();
        robot.ConfigureVision();
        robot.resetMotorsForAutonomous(robot.left_back, robot.left_front, robot.right_back, robot.right_front);
        robot.team=-1;
        robot.status("ready");
        waitForStart();
        robot.servo_2.setPosition(robot.r2_out);
        robot.DriveForward(DRIVE_SPEED,10, 1);
        robot.TurnByImu(TURN_SPEED,-90*robot.team,2);
        robot.servo_2.setPosition(robot.r2_in);
        robot.servo_3.setPosition(robot.l3_in);

        robot.DriveForward(DRIVE_SPEED,20,2);


    }

}
