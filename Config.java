package org.firstinspires.ftc.teamcode;

//import com.disnodeteam.dogecv.CameraViewDisplay;
//import com.disnodeteam.dogecv.DogeCV;
//import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.opencv.core.Size;
import java.util.Locale;


public class Config {
    Servo Servo_A;
    DcMotor left_front, right_front, left_back, right_back;
    // A timer object
    private ElapsedTime timer = new ElapsedTime();
    private final int EncoderNumberChangePerInch = 34;
    // Get the important bits from the opMode
    private LinearOpMode OpMode;

    Config(LinearOpMode OpMode) {
        this.OpMode = OpMode;
    }

    void ConfigureRobtHardware() {
        // Declare and setup left_front
        this.status("Configuring left front motor");
        this.left_front = OpMode.hardwareMap.dcMotor.get("left front");
        this.left_front.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.left_front.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.left_front.setMode(RunMode.RUN_USING_ENCODER);
        this.left_front.setDirection(Direction.FORWARD);

        // Declare and setup right_front
        this.status("Configuring right front motor");
        this.right_front = OpMode.hardwareMap.dcMotor.get("right front");
        this.right_front.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.right_front.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.right_front.setMode(RunMode.RUN_USING_ENCODER);
        this.right_front.setDirection(Direction.REVERSE);

        // Declare and setup left_back
        this.status("Configuring left back motor");
        this.left_back = OpMode.hardwareMap.dcMotor.get("left back");
        this.left_back.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.left_back.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.left_back.setMode(RunMode.RUN_USING_ENCODER);
        this.left_back.setDirection(Direction.FORWARD);

        // Declare and setup right_back
        this.status("Configuring right back motor");
        this.right_back = OpMode.hardwareMap.dcMotor.get("right back");
        this.right_back.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.right_back.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.right_back.setMode(RunMode.RUN_USING_ENCODER);
        this.right_back.setDirection(Direction.REVERSE);


        // Declare the servo
        this.status("Setting up left servo");
        this.Servo_A = OpMode.hardwareMap.servo.get("Servo A");
        this.status("Done!");
    }

    void resetMotorsForAutonomous(DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(0);
            motor.setMode(RunMode.RUN_TO_POSITION);
        }
    }

    private boolean isThere(int error, DcMotor... motors) {
        boolean reached = false;
        for (DcMotor motor : motors) {
            int delta = Math.abs(motor.getTargetPosition() - motor.getCurrentPosition());
            if (delta <= error) {
                reached = true;
                break;
            }
        }
        return reached;
    }
    void distinctDrive(double speed, double LFInches, double LBInches, double RFInches, double RBInches, double timeoutS) {

        // Reset the motor encoders, and set them to RUN_TO_POSITION
        this.resetMotorsForAutonomous(this.left_front, this.left_back, this.right_front, this.right_back);

        // Set the individual drive motor positions
        this.left_front.setTargetPosition((int) Math.round(LFInches * EncoderNumberChangePerInch));
        this.right_front.setTargetPosition((int) Math.round(RFInches * EncoderNumberChangePerInch));
        this.left_back.setTargetPosition((int) Math.round(LBInches * EncoderNumberChangePerInch));
        this.right_back.setTargetPosition((int) Math.round(RBInches * EncoderNumberChangePerInch));

        // Set the motor speeds
        this.left_front.setPower(speed);
        this.right_front.setPower(speed);
        this.left_back.setPower(speed);
        this.right_back.setPower(speed);

        // Reset the runtime
        this.timer.reset();
        while (OpMode.opModeIsActive() && (this.timer.seconds() < timeoutS)) {

            // Check if the target has been reached
            if (this.isThere(4, this.left_back, this.left_front, this.right_back, this.right_front)) {
                // Break out of the while loop early
                break;
            }
        }
        // Stop all motion, and reset the motors
        this.resetMotorsForAutonomous(this.left_back, this.left_front, this.right_back, this.right_front);
    }


    //I am not sure what is the code below doing
    void status(String string) {
        this.OpMode.telemetry.addData("Status", string);
        this.OpMode.telemetry.update();
    }
}
