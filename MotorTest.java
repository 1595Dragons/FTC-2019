

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "MotorTest", group = "Official")
public class MotorTest extends LinearOpMode {

    private Config7935 robot = new Config7935(this);

    public void runOpMode() {

        // Initialize the robot
        robot.ConfigureRobtHardware();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a){
                robot.left_front.setPower(0.2);
            }
            if (gamepad1.b){
                robot.left_back.setPower(0.2);
            }
            if (gamepad1.x){
                robot.right_front.setPower(0.2);
            }
            if (gamepad1.y){
                robot.right_back.setPower(0.2);
            }


            telemetry.addData("Lf power", robot.left_front.getPower())
                    .addData("Lf encoder",robot.left_front.getCurrentPosition())
                    .addData("Rf power", robot.right_front.getPower())
                    .addData("Rf encoder",robot.right_front.getCurrentPosition())
                    .addData("Lb power", robot.left_back.getPower())
                    .addData("Lb encoder",robot.left_back.getCurrentPosition())
                    .addData("Rb power", robot.right_back.getPower())
                    .addData("Rb encoder",robot.right_back.getCurrentPosition());
            telemetry.update();

        }
    }
}