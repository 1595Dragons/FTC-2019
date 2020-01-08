package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "B1AutoMoveFoundation7935", group = "Official")
//@Disabled
public class B1AutoMoveFoundation7935 extends LinearOpMode {


    //basic set up  code for search: 1001
    private static final double DRIVE_SPEED = .5, TURN_SPEED = .5, ARM_SPEED = .8, SIDE_SPEED = .4;
    private ElapsedTime runtime = new ElapsedTime();
    private Config7935 robot = new Config7935(this);

    public void runOpMode() {

        //basic set up code1001
        robot.ConfigureRobtHardware();
        //robot.ConfigureVision();
        robot.resetMotorsForAutonomous(robot.left_back, robot.left_front, robot.right_back, robot.right_front);

        robot.status("ready");
        waitForStart();
        robot.DriveForward(DRIVE_SPEED,8,1);
        robot.DriveLeft(SIDE_SPEED,-14*robot.team,3);
        robot.TurnByImu(TURN_SPEED,0,1);





        runtime.reset();
        robot.lift.setPower(-1);//negative is up

        while (opModeIsActive() && (runtime.seconds() < 0.6)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.lift.setPower(0);

        robot.DriveForward(DRIVE_SPEED,20,2);
        robot.TurnByImu(TURN_SPEED,0,1);
        robot.DriveForward(0.2,10,2);
        while(robot.lift_sensor.getState()==true){
            robot.lift.setPower(0.7);
        }
        robot.lift.setPower(0);

        robot.DriveForward(DRIVE_SPEED,-40,4);
        robot.lift.setPower(-0.8);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.lift.setPower(0);
        robot.DriveLeft(SIDE_SPEED,35*robot.team,3);

        robot.TurnByImu(TURN_SPEED,90*robot.team,2);
        while(robot.lift_sensor.getState()==true){
            robot.lift.setPower(0.8);
        }
        robot.lift.setPower(0);
        robot.TurnByImu(TURN_SPEED,90*robot.team,1);
        robot.DriveForward(DRIVE_SPEED,30,3);

        //robot.targetsSkyStone.deactivate();


    }

}

