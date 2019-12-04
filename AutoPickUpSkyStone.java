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

@Autonomous(name = "PickUpSkyStone", group = "Official")
//@Disabled
public class AutoPickUpSkyStone extends LinearOpMode {


    //basic set up  code for search: 1001
    private static final double DRIVE_SPEED = .9, TURN_SPEED = .9, ARM_SPEED = .8, SIDE_SPEED = .8;

    private Config robot = new Config(this);

    public void runOpMode() {

        //basic set up code1001
        robot.ConfigureRobtHardware();
        robot.ConfigureVision();
        robot.resetMotorsForAutonomous(robot.left_back, robot.left_front, robot.right_back, robot.right_front);

        robot.status("ready");
        waitForStart();

        robot.targetsSkyStone.activate();
        double visionX=0, visionY=0, visionTurn=0;
        int tryVision=0;
        robot.DriveForward(1,27,3);
        robot.DriveLeft(SIDE_SPEED,2.5,1);
        visionY=robot.lookForStoneY(2);

        if (visionY==998){
            robot.DriveLeft(SIDE_SPEED, -8.5,2);
            visionY=robot.lookForStoneY(2);
            if (visionY==998){
                robot.DriveLeft(SIDE_SPEED, -9,2);
            }
        }
        robot.DriveForward(1,-10,2);
        sleep(10000);

        robot.intake_left.setPower(1);
        robot.intake_right.setPower(1);
        robot.DriveForward(DRIVE_SPEED,25,3);
        sleep(3000);
        robot.intake_left.setPower(0);
        robot.intake_right.setPower(0);
        robot.DriveForward(0.9,-15,2);
        robot.TurnByImu(0.9,-90,2);
        robot.DriveForward(1,50,4);



        robot.targetsSkyStone.deactivate();


    }

}