package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AutoDriveByDistanceSensor7935", group = "Official")
//@Disabled
    public class AutoDriveByDistanceSensor7935 extends LinearOpMode {


        //basic set up  code for search: 1001
        private static final double DRIVE_SPEED = .5, TURN_SPEED = .5, ARM_SPEED = .8, SIDE_SPEED = .5;

        private Config7935 robot = new Config7935(this);

        public void runOpMode() {

            double error=0;
            double setPosition=30;
            //basic set up code1001
            robot.ConfigureRobtHardware();
            robot.ConfigureVision();
            robot.resetMotorsForAutonomous(robot.left_back, robot.left_front, robot.right_back, robot.right_front);

            robot.status("ready");
            waitForStart();

            robot.DriveForward(DRIVE_SPEED,-35,3);


            while(robot.distance_sensor.getDistance(DistanceUnit.INCH)>40){
                robot.DriveForward(DRIVE_SPEED,-35,3);

            }
            telemetry.addData("position",robot.distance_sensor.getDistance(DistanceUnit.INCH));
            error=setPosition-robot.distance_sensor.getDistance(DistanceUnit.INCH);
            robot.DriveForward(DRIVE_SPEED,error,3);
            telemetry.addData("error",error)
                    .addData("final position",robot.distance_sensor.getDistance(DistanceUnit.INCH));
            telemetry.update();


        }



}

