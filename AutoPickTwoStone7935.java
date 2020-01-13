package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AutoPickTwoStone7935", group = "official")
//@Disabled
public class AutoPickTwoStone7935 extends LinearOpMode {


    //basic set up  code for search: 1001
    private static final double DRIVE_SPEED = .5, TURN_SPEED = .5, ARM_SPEED = .8, SIDE_SPEED = .45;
    private ElapsedTime runtime = new ElapsedTime();
    private Config7935 robot = new Config7935(this);

    public void runOpMode() {

        //basic set up code1001
        robot.ConfigureRobtHardware();
        robot.ConfigureVision();
        double visionY=0,distance;
        int skyStonePosition=0;
        robot.resetMotorsForAutonomous(robot.left_back, robot.left_front, robot.right_back, robot.right_front);
        robot.status("ready");
        waitForStart();
        robot.targetsSkyStone.activate();
        robot.distinctDrivePlus(DRIVE_SPEED,-29,-29,-29,-29,2.5,-1,0.5);
        visionY=robot.lookForStoneY(0.5);
        skyStonePosition=1;
        if (visionY==998){
            robot.DriveLeft(SIDE_SPEED,10*robot.team,1);
            skyStonePosition=0;
            robot.TurnByImu(TURN_SPEED,0,0.5);
            visionY=robot.lookForStoneY(0.5);
            if (visionY==998){
                skyStonePosition=2;
                robot.DriveLeft(SIDE_SPEED, -22*robot.team,2);
                robot.TurnByImu(TURN_SPEED,0,0.5);
            }
        }

        //found stone or pick up the last one
        robot.distinctDrivePlus(0.4,10,10,10,10,1.5,1,1.5);
        robot.intake_left.setPower(1);
        robot.intake_right.setPower(1);
        robot.DriveForward(DRIVE_SPEED,27,2);
        robot.intake_left.setPower(0);
        robot.intake_right.setPower(0);
        robot.DriveForward(DRIVE_SPEED,-22,2.2);



        //travel across field
        robot.TurnByImu(TURN_SPEED,-90*robot.team,1);
        robot.DriveForward(1,55-skyStonePosition*8,3);
        robot.intake_left.setPower(-1);
        robot.intake_right.setPower(-1);
        robot.TurnByImu(TURN_SPEED,-90*robot.team,0.5);
        robot.DriveForward(1,-65,4);
        robot.intake_left.setPower(0);
        robot.intake_right.setPower(0);
        distance=robot.distance_sensor.getDistance(DistanceUnit.INCH);
        if(skyStonePosition==0){
            robot.DriveForward(DRIVE_SPEED,-distance+8,1.5);
            robot.TurnByImu(TURN_SPEED,15*robot.team,1.5);
            robot.intake_left.setPower(1);
            robot.intake_right.setPower(1);
            robot.DriveForward(DRIVE_SPEED,23,2);
            robot.intake_left.setPower(0);
            robot.intake_right.setPower(0);
            robot.DriveForward(DRIVE_SPEED,-25,2.2);
        }else {
            robot.DriveForward(DRIVE_SPEED,-distance+skyStonePosition*8,1.5);
            robot.TurnByImu(TURN_SPEED,0,1);
            robot.intake_left.setPower(1);
            robot.intake_right.setPower(1);
            robot.DriveForward(DRIVE_SPEED,21,2);
            robot.intake_left.setPower(0);
            robot.intake_right.setPower(0);
            robot.DriveForward(DRIVE_SPEED,-24,2.2);
        }
        robot.TurnByImu(TURN_SPEED,-90*robot.team,1);
        robot.DriveForward(1,75-skyStonePosition*8,3);
        robot.intake_left.setPower(-1);
        robot.intake_right.setPower(-1);
        robot.DriveForward(.7,-20,2);
        robot.intake_left.setPower(0);
        robot.intake_right.setPower(0);



        robot.targetsSkyStone.deactivate();
    }

}
