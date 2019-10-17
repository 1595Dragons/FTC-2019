package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "7935 TeleOp", group = "Official")
public class BasicTeleop extends LinearOpMode {

    private Config robot = new Config(this);

    public void runOpMode() {

        // Initialize the robot
        robot.ConfigureRobtHardware();
        //Motor Power
        double speedForTurn = 0.6, speedForMove = 0.7, speedForSide = 0.8, intakePower = 1, armPower = 1, extendPower = 1;
        double allPower = 0.7;
        double powerTurn = 1.5, powerMove=1.8,powerSide=1.9;
        boolean powerUp=false;
        waitForStart();

        while (opModeIsActive()) {

            //Power up function
            if (gamepad2.right_trigger>0){
                powerUp=true;
            }else{
                powerUp=false;
            }
            double driveForward = gamepad2.left_stick_y * speedForMove, driveRightSide = gamepad2.left_stick_x * speedForSide,
                    turnRight = -gamepad2.right_stick_x * speedForTurn;
            //prevent small input from stick
            driveForward = (driveForward >= -0.1 && driveForward <= 0.1) ? 0 : driveForward;
            driveRightSide = (driveRightSide >= -0.1 && driveRightSide <= 0.1) ? 0 : driveRightSide;
            turnRight = (turnRight >= -0.1 && turnRight <= 0.1) ? 0 : turnRight;

            if (gamepad2.dpad_left){
                driveRightSide=-1*speedForSide;
            }
            if (gamepad2.dpad_right){
                driveRightSide=1*speedForSide;
            }
            if(gamepad2.dpad_up){
                driveForward=1*speedForMove;
            }
            if(gamepad2.dpad_down){
                driveForward=-1*speedForMove;
            }

            // Send calculated power to wheels
            if (powerUp){
                robot.left_front.setPower(Range.clip((-driveRightSide*powerSide + driveForward*powerMove + turnRight*powerTurn) * allPower, -1.0, 1.0));
                robot.right_front.setPower(Range.clip((driveRightSide*powerSide + driveForward*powerMove - turnRight*powerTurn) * allPower, -1.0, 1.0));
                robot.left_back.setPower(Range.clip((driveRightSide*powerSide + driveForward*powerMove + turnRight*powerTurn) * allPower, -1.0, 1.0));
                robot.right_back.setPower(Range.clip((-driveRightSide*powerSide + driveForward*powerMove - turnRight*powerTurn) * allPower, -1.0, 1.0));
            }else{
                robot.left_front.setPower(Range.clip((-driveRightSide + driveForward + turnRight) * allPower, -1.0, 1.0));
                robot.right_front.setPower(Range.clip((driveRightSide + driveForward - turnRight) * allPower, -1.0, 1.0));
                robot.left_back.setPower(Range.clip((driveRightSide + driveForward + turnRight) * allPower, -1.0, 1.0));
                robot.right_back.setPower(Range.clip((-driveRightSide + driveForward - turnRight) * allPower, -1.0, 1.0));
            }


            /*
            if (gamepad1.a){
                robot.Servo_A.setPosition(0.5);
            }
            if (gamepad1.b){
                robot.Servo_A.setPosition(0);
            }
            */

            telemetry.addData("Lf power", robot.left_front.getPower())
                    .addData("Rf power", robot.right_front.getPower())
                    .addData("Lb power", robot.left_back.getPower())
                    .addData("Rb power", robot.right_back.getPower());
            telemetry.update();

        }
    }
}
