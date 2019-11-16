package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp6128", group = "Official")
public class TeleOp6128 extends LinearOpMode {

    private Config6128 robot = new Config6128(this);

    public void runOpMode() {

        // Initialize the robot
        robot.ConfigureRobtHardware();
        //Motor Power
        double speedTurn=1,speedMove=1,speedSide=1;
        double slowTurn = 0.5, slowMove=0.5,slowSide=0.7;
        boolean slowMode=false, powerMode=false;
        double powerUp=1.3;
        waitForStart();

        while (opModeIsActive()) {

            //Power up function
            if (gamepad2.left_trigger>0){
                slowMode=true;
            }else{
                slowMode=false;
            }
            if (gamepad2.right_trigger>0){
                powerMode=true;
                slowMode=false;
            }else{
                powerMode=false;
            }

            double driveForward = gamepad2.left_stick_y * speedMove, driveRightSide = gamepad2.left_stick_x * speedSide,
                    turnRight = -gamepad2.right_stick_x * speedTurn;
            //prevent small input from stick
            driveForward = (driveForward >= -0.1 && driveForward <= 0.1) ? 0 : driveForward;
            driveRightSide = (driveRightSide >= -0.1 && driveRightSide <= 0.1) ? 0 : driveRightSide;
            turnRight = (turnRight >= -0.1 && turnRight <= 0.1) ? 0 : turnRight;

            if (gamepad2.dpad_left){
                driveRightSide=-1*speedSide;
            }
            if (gamepad2.dpad_right){
                driveRightSide=1*speedSide;
            }
            if(gamepad2.dpad_up){
                driveForward=1*speedMove;
            }
            if(gamepad2.dpad_down){
                driveForward=-1*speedMove;
            }

            // Send calculated power to wheels
            if (slowMode){
                robot.left_front.setPower(Range.clip((-driveRightSide*slowSide + driveForward*slowMove + turnRight*slowTurn), -1.0, 1.0));
                robot.right_front.setPower(Range.clip((driveRightSide*slowSide + driveForward*slowMove - turnRight*slowTurn), -1.0, 1.0));
                robot.left_back.setPower(Range.clip((driveRightSide*slowSide + driveForward*slowMove + turnRight*slowTurn), -1.0, 1.0));
                robot.right_back.setPower(Range.clip((-driveRightSide*slowSide + driveForward*slowMove - turnRight*slowTurn), -1.0, 1.0));
            }else if(!powerMode){
                robot.left_front.setPower(Range.clip((-driveRightSide + driveForward + turnRight), -1.0, 1.0));
                robot.right_front.setPower(Range.clip((driveRightSide + driveForward - turnRight), -1.0, 1.0));
                robot.left_back.setPower(Range.clip((driveRightSide + driveForward + turnRight), -1.0, 1.0));
                robot.right_back.setPower(Range.clip((-driveRightSide + driveForward - turnRight), -1.0, 1.0));
            }else{
                robot.left_front.setPower(Range.clip((driveRightSide - driveForward + turnRight), -1.0, 1.0));
                robot.right_front.setPower(Range.clip((-driveRightSide - driveForward - turnRight), -1.0, 1.0));
                robot.left_back.setPower(Range.clip((-driveRightSide - driveForward + turnRight), -1.0, 1.0));
                robot.right_back.setPower(Range.clip((driveRightSide - driveForward - turnRight), -1.0, 1.0));
            }



            if (gamepad1.a){
                robot.servo_0.setPosition(1);
                robot.servo_1.setPosition(0);
            }
            if (gamepad1.b){
                robot.servo_0.setPosition(0.5);
                robot.servo_1.setPosition(0.4);
            }
            if (gamepad1.x){
                robot.servo_0.setPosition(0);
                robot.servo_1.setPosition(1);
            }
            if (gamepad1.y){
                telemetry.addData("R0 servo",robot.servo_0.getPosition())
                        .addData("L1 servo",robot.servo_1.getPosition());
            }
           // robot.servo_1.setPosition(gamepad1.left_trigger);
            //robot.servo_0.setPosition(gamepad1.right_trigger);




            telemetry.addData("Lf power", robot.left_front.getPower())
                    .addData("Rf power", robot.right_front.getPower())
                    .addData("Lb power", robot.left_back.getPower())
                    .addData("Rb power", robot.right_back.getPower())
                    .addData("R0 servo",robot.servo_0.getPosition())
                    .addData("L1 servo",robot.servo_1.getPosition());
            telemetry.update();

        }
    }
}
