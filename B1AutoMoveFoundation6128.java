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

@Autonomous(name = "B1AutoMoveFoundation6128", group = "Official")
//@Disabled
public class B1AutoMoveFoundation6128 extends LinearOpMode {
    //start position B1, park outside along the wall


    //basic set up  code for search: 1001
    private static final double DRIVE_SPEED = .4, TURN_SPEED = .4, SIDE_SPEED = .3;

    private Config6128 robot = new Config6128(this);

    public void runOpMode() {
        //basic set up code1001
        robot.ConfigureRobtHardware();
        robot.resetMotorsForAutonomous(robot.left_back, robot.left_front, robot.right_back, robot.right_front);

        robot.status("ready");
        waitForStart();

        robot.DriveForward(DRIVE_SPEED, 6, 2);
        sleep(1000);
        robot.DriveLeft(SIDE_SPEED, 40, 3);
        sleep(1000);
        robot.TurnByImu(TURN_SPEED, 0 * robot.team, 2);
        sleep(1000);
        robot.DriveForward(DRIVE_SPEED, 25, 3);
        sleep(1000);
        robot.servo_0.setPosition(robot.r0_down);
        robot.servo_1.setPosition(robot.l1_down);
        sleep(3000);
        robot.DriveForward(DRIVE_SPEED, -25, 3);
        robot.servo_0.setPosition(robot.r0_middle);
        robot.servo_1.setPosition(robot.l1_middle);
        sleep(1000);
        robot.TurnByImu(TURN_SPEED, 90 * robot.team, 2);
        sleep(1000);
        robot.DriveForward(DRIVE_SPEED, 40, 4);
        sleep(1000);
        robot.DriveLeft(SIDE_SPEED, 4 * robot.team, 2);
        sleep(1000);


        }
    }
