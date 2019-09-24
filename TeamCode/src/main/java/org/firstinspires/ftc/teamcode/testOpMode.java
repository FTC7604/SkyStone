package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Motor.HumanController;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode;

import static java.lang.Math.abs;

@TeleOp(name="Test OpMode", group="Linear Opmode")
public class testOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    double[] driveTrainController = new double[3];
    double intakePower = 0;
    double armPower = 0;

    double armPosition = 0;
    boolean blockIntakeTouchSensor = true;

    @Override
    public void runOpMode() {
        /*The robot objects is all of the motors, and sensors that are within the robot,
        essentially the code framework of the robot*/
        RobotLinearOpMode robotLinearOpMode = new RobotLinearOpMode(this);

        robotLinearOpMode.setDriveTrainRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotLinearOpMode.setDriveTrainZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);

        robotLinearOpMode.setIntakeRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotLinearOpMode.setIntakeZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);

        robotLinearOpMode.setArmRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotLinearOpMode.setIntakeZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //this is when the robot receives the 'play' command
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //sets up the condidtion for the drivetrain
            driveTrainController[1] = HumanController.humanController(((-gamepad1.right_stick_y)*(abs(-gamepad1.right_stick_y))+((-gamepad1.left_stick_y)*(abs(-gamepad1.left_stick_y))))/2);
            driveTrainController[0] = - HumanController.humanController(((-gamepad1.right_stick_x)*(abs(-gamepad1.right_stick_x))+((-gamepad1.left_stick_x)*(abs(-gamepad1.left_stick_x))))/2);
            driveTrainController[2] = HumanController.humanController(((-gamepad1.right_stick_y)-(-gamepad1.left_stick_y))/2);

            //increments the intake power
            intakePower = gamepad2.left_stick_y;

            armPower = gamepad2.right_stick_y;

            //sends this to the motors.
            robotLinearOpMode.mecPowerDrive(driveTrainController);
            robotLinearOpMode.setIntakePower(intakePower);
            //robotLinearOpMode.setArmPower(armPower);

            //gets the position arm encoder
            armPosition = robotLinearOpMode.getArmEncoder();
            blockIntakeTouchSensor = robotLinearOpMode.blockIntakeTouchSensorIsPressed();

            //now sends it too teleOp
            telemetry.addData("Intake Movement: ", intakePower);
            telemetry.addData("Arm Position: ", armPosition);
            telemetry.addData("Intake Touch Boolean: ", blockIntakeTouchSensor);
            telemetry.update();

        }
    }
}

