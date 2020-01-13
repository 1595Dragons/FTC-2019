package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AutoPutStoneAndMoveFoundation7935", group = "official")
//@Disabled
public class AutoPutStoneAndMoveFoundation7935 extends LinearOpMode {


    //basic set up  code for search: 1001
    private static final double DRIVE_SPEED = .5, TURN_SPEED = .5, ARM_SPEED = .8, SIDE_SPEED = .45;
    private ElapsedTime runtime = new ElapsedTime();
    private Config7935 robot = new Config7935(this);

    public void runOpMode() {

        //basic set up code1001
        robot.ConfigureRobtHardware();
        robot.ConfigureVision();
        double visionY=0,distance;
        robot.resetMotorsForAutonomous(robot.left_back, robot.left_front, robot.right_back, robot.right_front);
        robot.status("ready");
        waitForStart();
        robot.targetsSkyStone.activate();
        robot.distinctDrivePlus(DRIVE_SPEED,-29,-29,-29,-29,2.5,-1,0.5);
        robot.TurnByImu(TURN_SPEED,0,0.5);
        visionY=robot.lookForStoneY(0.5);
        if (visionY==998){
            robot.DriveLeft(SIDE_SPEED,10*robot.team,1);
            robot.TurnByImu(TURN_SPEED,0,0.5);
            visionY=robot.lookForStoneY(0.5);
            if (visionY==998){
                robot.DriveLeft(SIDE_SPEED, -21*robot.team,2);
                robot.TurnByImu(TURN_SPEED,0,0.5);
            }
        }

        //found stone or pick up the last one
        robot.distinctDrivePlus(0.3,10,10,10,10,1.5,1,1.5);

        robot.intake_left.setPower(1);
        robot.intake_right.setPower(1);
        robot.DriveForward(DRIVE_SPEED,27,2);
        //sleep(500);
        robot.intake_left.setPower(0);
        robot.intake_right.setPower(0);
        robot.DriveForward(DRIVE_SPEED,-20,2);


        //travel across field
        robot.TurnByImu(TURN_SPEED,90*robot.team,1);
        robot.TurnByImu(TURN_SPEED,90*robot.team,0.5);
        robot.DriveForward(DRIVE_SPEED,-75,4);
        distance=robot.distance_sensor.getDistance(DistanceUnit.INCH);
        //telemetry.addData("distance",distance);
        //telemetry.update();
        //robot.DriveForward(DRIVE_SPEED,13-distance,1);
        robot.distinctDrivePlus(DRIVE_SPEED,distance-13,distance-13,distance-13,distance-13,1,-1,0.5);
        robot.TurnByImu(TURN_SPEED,0,1);

        robot.distinctDrivePlus(DRIVE_SPEED,-14,-14,-14,-14,2,-1,0.5);
        robot.intake_left.setPower(-1);
        robot.intake_right.setPower(-1);
        sleep(300);
        while(robot.lift_sensor.getState()==true){
            robot.lift.setPower(1);
        }
        robot.lift.setPower(0);
        robot.intake_left.setPower(0);
        robot.intake_right.setPower(0);
        robot.DriveForward(DRIVE_SPEED,-43,2);
        runtime.reset();
        robot.lift.setPower(-1);
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.lift.setPower(0);
        //robot.TurnByImu(0.3,90*robot.team,2);
        /*
        robot.DriveLeft(SIDE_SPEED,35*robot.team,2.5);
        robot.TurnByImu(TURN_SPEED,90*robot.team,1);
        robot.distinctDrivePlus(0.6,-25,-25,-25,-25,1.5,1,1);
        //robot.distinctDrivePlus(0.7,25,-25,-25,25,2,1,1);

         */
        robot.targetsSkyStone.deactivate();
    }

}