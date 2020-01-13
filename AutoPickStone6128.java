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

@Autonomous(name = "2020PickStone6128", group = "Official")
//@Disabled
public class AutoPickStone6128 extends LinearOpMode {

    private static final double DRIVE_SPEED = .4, TURN_SPEED = .4, SIDE_SPEED = .3;
    private ElapsedTime runtime = new ElapsedTime();
    private Config6128 robot = new Config6128(this);

    public void runOpMode() {
        robot.ConfigureRobtHardware();
        robot.resetMotorsForAutonomous(robot.left_back, robot.left_front, robot.right_back, robot.right_front);
        robot.status("ready");
        waitForStart();
        //initialize servo
        robot.servo_2.setPosition(robot.r2_out);
        robot.servo_3.setPosition(robot.l3_out);
        robot.lift_left.setPower(-0.4);
        robot.lift_right.setPower(-0.4);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.lift_left.setPower(0);
        robot.lift_right.setPower(0);
        robot.DriveForward(DRIVE_SPEED, 25, 2);
        robot.intake_left.setPower(-1);
        robot.intake_right.setPower(-1);
        robot.DriveForward(DRIVE_SPEED,15,2);
        robot.servo_2.setPosition(robot.r2_in);
        robot.servo_3.setPosition(robot.l3_in);
        sleep(500);
        robot.intake_left.setPower(0);
        robot.intake_right.setPower(0);
        robot.servo_2.setPosition(robot.r2_squ);
        robot.servo_3.setPosition(robot.l3_squ);
        robot.DriveForward(DRIVE_SPEED,-20,3);
        robot.TurnByImu(TURN_SPEED,-90*robot.team,1.5);
        robot.DriveForward(DRIVE_SPEED,45,2);
        robot.LiftUp(0.5,0.8);
        robot.DriveForward(DRIVE_SPEED,10,1);
        robot.servo_2.setPosition(robot.r2_out);
        robot.servo_3.setPosition(robot.l3_out);
        robot.intake_left.setPower(1);
        robot.intake_right.setPower(1);
        robot.DriveForward(DRIVE_SPEED,-10,1);
        robot.LiftUp(-0.5,0.8);
        robot.intake_left.setPower(0);
        robot.intake_right.setPower(0);
        robot.DriveForward(DRIVE_SPEED,-76,2.5);

        //second one
        robot.TurnByImu(TURN_SPEED,0,1.5);

        robot.intake_left.setPower(-1);
        robot.intake_right.setPower(-1);
        robot.DriveForward(DRIVE_SPEED,20,2);
        robot.servo_2.setPosition(robot.r2_in);
        robot.servo_3.setPosition(robot.l3_in);
        sleep(500);
        robot.intake_left.setPower(0);
        robot.intake_right.setPower(0);
        robot.servo_2.setPosition(robot.r2_squ);
        robot.servo_3.setPosition(robot.l3_squ);
        robot.DriveForward(DRIVE_SPEED,-20,3);

        robot.TurnByImu(TURN_SPEED,-90*robot.team,1.5);
        robot.DriveForward(DRIVE_SPEED,76,2.5);
        robot.LiftUp(0.4,1.2);
        robot.DriveForward(DRIVE_SPEED,10,1);
        robot.servo_2.setPosition(robot.r2_out);
        robot.servo_3.setPosition(robot.l3_out);
        robot.intake_left.setPower(1);
        robot.intake_right.setPower(1);
        robot.DriveForward(DRIVE_SPEED,-10,1);
        robot.LiftUp(-0.4,1.2);
        robot.DriveForward(DRIVE_SPEED,-15,2.5);
        robot.intake_left.setPower(0);
        robot.intake_right.setPower(0);
    }
}

