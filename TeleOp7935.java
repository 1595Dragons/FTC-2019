package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp7935", group = "Official")
public class TeleOp7935 extends LinearOpMode {

    private Config7935 robot = new Config7935(this);

    public void runOpMode() {

        // Initialize the robot
        robot.ConfigureRobtHardware();
        //Motor Power

        double speedTurn=0.7,speedMove=0.7,speedSide=0.7;
        double slowTurn = 0.4, slowMove=0.4,slowSide=0.4;//multiply with normal speed
        double powerTurn=1.43,powerMove=1.43,powerSide=1.43;
        boolean slowMode=false, powerMode=false;
        double intakePower=0,liftPower=0.6;


        waitForStart();

        while (opModeIsActive()) {

            //Power up function

            if (gamepad2.left_trigger>0.1){
                slowMode=true;
            }else{
                slowMode=false;
                if (gamepad2.right_trigger>0.1){

                    powerMode=true;
                }else{
                    powerMode=false;
                }
            }




            double driveForward = gamepad2.left_stick_y * speedMove, driveRightSide = gamepad2.left_stick_x * speedSide,
                    turnRight = -gamepad2.right_stick_x * speedTurn;

            intakePower=gamepad1.left_trigger-gamepad1.right_trigger;//left in right out?
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
            if (gamepad2.b){
                double bPowerOut=-0.4;
                robot.intake_left.setPower(bPowerOut);
                robot.intake_right.setPower(bPowerOut);
                double bPowerBack=0.15;
                robot.left_front.setPower(bPowerBack);
                robot.left_back.setPower(bPowerBack);
                robot.right_front.setPower(bPowerBack);
                robot.right_back.setPower(bPowerBack);
            }else if(gamepad2.a){
                double aPowerOut=0.4;
                robot.intake_left.setPower(aPowerOut);
                robot.intake_right.setPower(aPowerOut);
                double aPowerBack=-0.15;
                robot.left_front.setPower(aPowerBack);
                robot.left_back.setPower(aPowerBack);
                robot.right_front.setPower(aPowerBack);
                robot.right_back.setPower(aPowerBack);
            }
            else {
                if (slowMode){
                    robot.left_front.setPower(Range.clip((-driveRightSide*slowSide + driveForward*slowMove + turnRight*slowTurn), -1.0, 1.0));
                    robot.right_front.setPower(Range.clip((driveRightSide*slowSide + driveForward*slowMove - turnRight*slowTurn), -1.0, 1.0));
                    robot.left_back.setPower(Range.clip((driveRightSide*slowSide + driveForward*slowMove + turnRight*slowTurn), -1.0, 1.0));
                    robot.right_back.setPower(Range.clip((-driveRightSide*slowSide + driveForward*slowMove - turnRight*slowTurn), -1.0, 1.0));
                }else {
                    if (!powerMode) {
                        robot.left_front.setPower(Range.clip((-driveRightSide + driveForward + turnRight), -1.0, 1.0));
                        robot.right_front.setPower(Range.clip((driveRightSide + driveForward - turnRight), -1.0, 1.0));
                        robot.left_back.setPower(Range.clip((driveRightSide + driveForward + turnRight), -1.0, 1.0));
                        robot.right_back.setPower(Range.clip((-driveRightSide + driveForward - turnRight), -1.0, 1.0));
                    } else {
                        robot.left_front.setPower(Range.clip((-driveRightSide * powerSide + driveForward * powerMove + turnRight * powerTurn), -1.0, 1.0));
                        robot.right_front.setPower(Range.clip((driveRightSide * powerSide + driveForward * powerMove - turnRight * powerTurn), -1.0, 1.0));
                        robot.left_back.setPower(Range.clip((driveRightSide * powerSide + driveForward * powerMove + turnRight * powerTurn), -1.0, 1.0));
                        robot.right_back.setPower(Range.clip((-driveRightSide * powerSide + driveForward * powerMove - turnRight * powerTurn), -1.0, 1.0));
                    }
                }
                robot.intake_left.setPower(intakePower);
                robot.intake_right.setPower(intakePower);
            }
            /*
            if (robot.intake_button.getState()==false){
                intakePower=-gamepad1.right_trigger;
            }*/




            if (robot.lift_sensor.getState()==true){
                robot.lift.setPower(gamepad1.left_stick_y*liftPower);
            }else if (gamepad1.left_stick_y<-0.1){//up is negative for the stick y
                robot.lift.setPower(gamepad1.left_stick_y*liftPower);
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
            telemetry.addData("button",robot.intake_button.getState())
                    .addData("left stick y",gamepad1.left_stick_y)
                    .addData("sensor",robot.lift_sensor.getState());
            telemetry.update();

        }
    }
}