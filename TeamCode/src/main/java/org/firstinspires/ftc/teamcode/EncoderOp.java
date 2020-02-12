//imports the package so that all the code can be used
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.CaseyMotionProfile;
import org.firstinspires.ftc.teamcode.Control.HumanController;
import org.firstinspires.ftc.teamcode.Control.Toggle;
import org.firstinspires.ftc.teamcode.LED.LedPattern;
import org.firstinspires.ftc.teamcode.LED.LedPatternStep;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.DWAIAutonomous.ALLIANCE;
import static org.firstinspires.ftc.teamcode.DWAIAutonomous.ALLIANCE.BLUE;

@TeleOp(name = "Encoder Op", group = "Linear Opmode")
public class EncoderOp extends LinearOpMode {
    private double[] driveTrainController = new double[3];
    private HumanController humanController = new HumanController(0.1, 1);
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFrontDriveMotor;
    private DcMotor leftFrontDriveMotor;
    private DcMotor rightBackDriveMotor;
    private DcMotor leftBackDriveMotor;

    @Override
    public void runOpMode(){
        initHardware();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while(opModeIsActive()){
            runDrive();
            printEncoders();

            if(gamepad1.left_stick_button && gamepad1.right_stick_button){
                rightFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                telemetry.addLine("Finished resetting encoders");
                telemetry.update();
            }

        }
    }

    private void printEncoders(){
        telemetry.addData("RF", rightFrontDriveMotor.getCurrentPosition());
        telemetry.addData("LF", leftFrontDriveMotor.getCurrentPosition());
        telemetry.addData("RB", rightBackDriveMotor.getCurrentPosition());
        telemetry.addData("LB", leftBackDriveMotor.getCurrentPosition());
        telemetry.update();
    }

    private void initHardware(){
        //assigns it to the config
        rightFrontDriveMotor  = hardwareMap.get(DcMotor.class, "lf");
        leftFrontDriveMotor   = hardwareMap.get(DcMotor.class, "rf");
        rightBackDriveMotor   = hardwareMap.get(DcMotor.class, "lb");
        leftBackDriveMotor    = hardwareMap.get(DcMotor.class, "rb");

        //sets the direction
        rightFrontDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackDriveMotor.setDirection(DcMotor.Direction.FORWARD);

        //Brake
        rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Encoders
        rightFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void mecanumPowerDrive(double strafe, double forward, double rotation){
        leftFrontDriveMotor.setPower(forward - strafe + rotation);
        leftBackDriveMotor.setPower(forward + strafe + rotation);
        rightFrontDriveMotor.setPower(forward + strafe - rotation);
        rightBackDriveMotor.setPower(forward - strafe - rotation);
    }

    private void mecanumPowerDrive(double[] controller){
        mecanumPowerDrive(controller[0], controller[1], controller[2]);
    }

    private void runDrive(){
        driveTrainController[1] = -gamepad1.left_stick_y;
        driveTrainController[0] = gamepad1.left_stick_x;
        driveTrainController[2] = -gamepad1.right_stick_x;

        if (gamepad1.left_bumper) {
            driveTrainController[1] /= 3;
            driveTrainController[0] /= 2;
            driveTrainController[2] /= 3;
        }

        mecanumPowerDrive(driveTrainController);
    }


}