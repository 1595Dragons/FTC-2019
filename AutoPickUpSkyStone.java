/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "PickUpSkyStone", group = "Official")
//@Disabled
public class AutoPickUpSkyStone extends LinearOpMode {


    //basic set up  code for search: 1001
    private static final double DRIVE_SPEED = .5, TURN_SPEED = .5, ARM_SPEED = .8, SIDE_SPEED = .4;
    private ElapsedTime runtime = new ElapsedTime();
    private Config7935 robot = new Config7935(this);

    public void runOpMode() {

        //basic set up code1001
        robot.ConfigureRobtHardware();
        robot.ConfigureVision();
        robot.resetMotorsForAutonomous(robot.left_back, robot.left_front, robot.right_back, robot.right_front);

        robot.status("ready");
        waitForStart();

        robot.targetsSkyStone.activate();
        double visionX=0, visionY=0, visionTurn=0;
        int skystonePosition=1;


        robot.DriveForward(DRIVE_SPEED,29,2);
        visionY=robot.lookForStoneY(1);
        if (visionY==998){
            robot.DriveLeft(SIDE_SPEED,14*robot.team,2);
            skystonePosition=0;
            robot.TurnByImu(TURN_SPEED,0,0.5);
            visionY=robot.lookForStoneY(1);
            if (visionY==998){
                robot.DriveLeft(SIDE_SPEED, -20*robot.team,3);
                skystonePosition=2;
                robot.TurnByImu(TURN_SPEED,0,0.5);
            }
        }

        //found stone or pick up the last one
        runtime.reset();
        //robot.lift.setPower(-0.4);//negative is up
        robot.DriveForward(DRIVE_SPEED,-10,1);
        while(robot.lift_sensor.getState()==true){
            robot.lift.setPower(0.8);
        }
        robot.lift.setPower(0);
        /*
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        c
        */
        robot.intake_left.setPower(1);
        robot.intake_right.setPower(1);
        robot.DriveForward(DRIVE_SPEED,25,3);
        robot.intake_left.setPower(0);
        robot.intake_right.setPower(0);
        robot.DriveForward(DRIVE_SPEED,-15,2);

        //turn and go accross bridge
        robot.TurnByImu(DRIVE_SPEED,-90*robot.team,1.5);
        //robot.DriveForward(DRIVE_SPEED,45,3);
        //robot.TurnByImu(DRIVE_SPEED,-90*robot.team,0.5);
        robot.DriveForward(DRIVE_SPEED,90-skystonePosition*7,5);
        runtime.reset();
        robot.lift.setPower(-0.9);
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.lift.setPower(0);
        robot.TurnByImu(TURN_SPEED,0,1);
        robot.DriveForward(DRIVE_SPEED,11,1);
        robot.intake_left.setPower(-1);
        robot.intake_right.setPower(-1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        /*
        while(robot.lift_sensor.getState()==true){
            robot.lift.setPower(0.8);
        }
        robot.lift.setPower(0);
        */

        robot.intake_left.setPower(0);
        robot.intake_right.setPower(0);
        robot.DriveForward(DRIVE_SPEED,-40,4);
        /*
        robot.lift.setPower(-0.8);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }


        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.lift.setPower(0);

         */

        robot.TurnByImu(TURN_SPEED,90*robot.team,1);
        while(robot.lift_sensor.getState()==true){
            robot.lift.setPower(1);
        }
        robot.lift.setPower(0);
        robot.DriveForward(DRIVE_SPEED,55,3);




        robot.targetsSkyStone.deactivate();


    }

}