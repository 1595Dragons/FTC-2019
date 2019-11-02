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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
//@Disabled
@Autonomous(name = "AutoWithVision", group = "Test")

public class AutoWithVision extends LinearOpMode {


    //basic set up  code for search: 1001
    private static final double DRIVE_SPEED =1, TURN_SPEED=1, ARM_SPEED =1, SIDE_SPEED =0.7;

    private Config robot = new Config(this);

    @Override public void runOpMode() {

        //basic set up code1001
        robot.ConfigureRobtHardware();
        robot.ConfigureVision();
        robot.resetMotorsForAutonomous(robot.left_back, robot.left_front, robot.right_back, robot.right_front);
        robot.status("ready");
        waitForStart();

        robot.targetsSkyStone.activate();
        double visionX=0, visionY=0, visionTurn=0;
        robot.DriveForward(DRIVE_SPEED,23,2);
        visionY=robot.lookForStoneY(2);
        if (robot.stoneFind){
            telemetry.addData("stone","find");
            telemetry.update();
            robot.DriveLeft(SIDE_SPEED,visionY,2);
        }

        sleep(1000);
        robot.DriveForward(DRIVE_SPEED,45,3);
        robot.DriveForward(DRIVE_SPEED,-5,1);


        robot.TurnByImu(TURN_SPEED,-90,2);
        robot.DriveForward(DRIVE_SPEED,66.5,3);


        robot.TurnByImu(TURN_SPEED,-135,1.5);
        robot.DriveForward(0.7,11,2);
        robot.DriveForward(DRIVE_SPEED, -8,1.5);
        robot.TurnByImu(TURN_SPEED,-90,1.5);
        robot.DriveForward(DRIVE_SPEED,25,2);
        robot.TurnByImu(TURN_SPEED,-179,1.5);
        robot.DriveForward(DRIVE_SPEED,40,2);
        robot.TurnByImu(TURN_SPEED,90,2);
        robot.DriveForward(DRIVE_SPEED,50,3);
        /*
        robot.TurnByImu(TURN_SPEED,-170,2);
        robot.DriveForward(DRIVE_SPEED,30,2);
        robot.TurnByImu(TURN_SPEED,-170,2);
        robot.DriveForward(DRIVE_SPEED,25,2);
        */




        robot.targetsSkyStone.deactivate();


    }

}


