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
        double slowTurn = 0.3, slowMove=0.3,slowSide=0.3;
        double regTurn = 0.6, regMove=0.6,regSide=1;
        boolean slowMode=false, powerMode=false;
        double powerUp=1.3;
        //servo position/




        waitForStart();

        while (opModeIsActive()) {
            //lift test
            double liftPower=gamepad1.left_stick_y;
            if (gamepad1.left_stick_button){
                liftPower=liftPower*0.4;
            }

            robot.lift_right.setPower(liftPower);
            robot.lift_left.setPower(liftPower);


            //Power up function

            if (gamepad2.right_bumper){
                powerMode=true;
                slowMode=false;
            }else{
                powerMode=false;
                if (gamepad2.left_bumper){
                    slowMode=true;
                }else{
                    slowMode=false;
                }

            }

            double driveForward = gamepad2.left_stick_y * speedMove, driveRightSide = gamepad2.left_stick_x * speedSide,
                    turnRight = -gamepad2.right_stick_x * speedTurn;
            double intakePower=gamepad1.left_trigger-gamepad1.right_trigger;
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
            if (gamepad2.left_trigger<=0.1&&gamepad2.right_trigger<=0.1){
                if (slowMode){
                    robot.left_front.setPower(Range.clip((-driveRightSide*slowSide + driveForward*slowMove + turnRight*slowTurn), -1.0, 1.0));
                    robot.right_front.setPower(Range.clip((driveRightSide*slowSide + driveForward*slowMove - turnRight*slowTurn), -1.0, 1.0));
                    robot.left_back.setPower(Range.clip((driveRightSide*slowSide + driveForward*slowMove + turnRight*slowTurn), -1.0, 1.0));
                    robot.right_back.setPower(Range.clip((-driveRightSide*slowSide + driveForward*slowMove - turnRight*slowTurn), -1.0, 1.0));
                }else if(!powerMode){
                    robot.left_front.setPower(Range.clip((-driveRightSide*regSide + driveForward*regMove + turnRight*regTurn), -1.0, 1.0));
                    robot.right_front.setPower(Range.clip((driveRightSide*regSide + driveForward*regMove - turnRight*regTurn), -1.0, 1.0));
                    robot.left_back.setPower(Range.clip((driveRightSide*regSide + driveForward*regMove + turnRight*regTurn), -1.0, 1.0));
                    robot.right_back.setPower(Range.clip((-driveRightSide*regSide + driveForward*regMove - turnRight*regTurn), -1.0, 1.0));
                }else{
                    robot.left_front.setPower(Range.clip((-driveRightSide + driveForward + turnRight), -1.0, 1.0));
                    robot.right_front.setPower(Range.clip((driveRightSide + driveForward - turnRight), -1.0, 1.0));
                    robot.left_back.setPower(Range.clip((driveRightSide + driveForward + turnRight), -1.0, 1.0));
                    robot.right_back.setPower(Range.clip((-driveRightSide + driveForward - turnRight), -1.0, 1.0));
                }
            }else{
                if (gamepad2.left_trigger>=0.1){
                    robot.left_front.setPower(Range.clip((gamepad2.left_trigger), -1.0, 1.0));
                    robot.right_front.setPower(Range.clip((-gamepad2.left_trigger), -1.0, 1.0));
                    robot.left_back.setPower(Range.clip((-gamepad2.left_trigger), -1.0, 1.0));
                    robot.right_back.setPower(Range.clip((gamepad2.left_trigger), -1.0, 1.0));
                }
                if (gamepad2.right_trigger>=0.1){
                    robot.left_front.setPower(Range.clip((-gamepad2.right_trigger), -1.0, 1.0));
                    robot.right_front.setPower(Range.clip((gamepad2.right_trigger), -1.0, 1.0));
                    robot.left_back.setPower(Range.clip((gamepad2.right_trigger), -1.0, 1.0));
                    robot.right_back.setPower(Range.clip((-gamepad2.right_trigger), -1.0, 1.0));
                }
            }



            robot.intake_left.setPower(intakePower);
            robot.intake_right.setPower(intakePower);
            if (gamepad1.right_bumper){
                robot.servo_2.setPosition(robot.r2_squ);
                robot.servo_3.setPosition(robot.l3_squ);
            }

            if ((gamepad1.right_trigger>0)&&(gamepad1.right_bumper==false)){
                robot.servo_2.setPosition(robot.r2_in);
                robot.servo_3.setPosition(robot.l3_in);
            }
            if (gamepad1.left_bumper){
                robot.servo_2.setPosition(robot.r2_out);
                robot.servo_3.setPosition(robot.l3_out);
            }
            if (gamepad1.a){
                robot.servo_0.setPosition(robot.r0_down);
                robot.servo_1.setPosition(robot.l1_down);
            }
            if (gamepad1.b){
                robot.servo_0.setPosition(robot.r0_middle);
                robot.servo_1.setPosition(robot.l1_middle);
            }
            if(gamepad1.x){
                robot.servo_0.setPosition(robot.r0_up);
                robot.servo_1.setPosition(robot.l1_up);
            }
            //robot.servo_2.setPosition(gamepad1.left_trigger);
            //robot.servo_3.setPosition(gamepad1.right_trigger);

            telemetry.addData("left button",robot.left_button.getState())
                    .addData("right button",robot.right_button.getState());
            telemetry.addData("lift_right power",robot.lift_right.getPower())
                    .addData("lift_left power",robot.lift_left.getPower())
                    .addData("intake power",intakePower)
                    .addData("intake_right",robot.intake_right.getPower())
                    .addData("intake_left",robot.intake_left.getPower());
            telemetry.addData("Lf power", robot.left_front.getPower())
                    .addData("Rf power", robot.right_front.getPower())
                    .addData("Lb power", robot.left_back.getPower())
                    .addData("Rb power", robot.right_back.getPower());

            telemetry.addData("R2 servo",robot.servo_2.getPosition())
                    .addData("L3 servo",robot.servo_3.getPosition());
            telemetry.update();

        }
    }
}
