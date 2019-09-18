package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Motor.HumanController;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name="Test OpMode", group="Linear Opmode")
public class testOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    double[] driveTrainController = new double[3];
    double intakePower = 0;

    @Override
    public void runOpMode() {
        /*The robot objects is all of the motors, and sensors that are within the robot,
        essentially the code framework of the robot*/
        Robot robot = new Robot(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //this is when the robot receives the 'play' command
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //sets up the condidtion for the drivetrain
            driveTrainController[0] = HumanController.humanController(gamepad1.left_stick_x);
            driveTrainController[1] = HumanController.humanController(-gamepad1.left_stick_y);
            driveTrainController[2] = HumanController.humanController(-gamepad1.right_stick_x);

            //increments the intake power
            intakePower = gamepad1.right_trigger;

            //sends this to the motors.
            robot.mecPowerDrive(driveTrainController);
            robot.setIntakePower(intakePower);

            //now sends it too teleOp
            telemetry.addData("X Movement:", driveTrainController[0]);
            telemetry.addData("Y Movement:", driveTrainController[1]);
            telemetry.addData("R Movement:", driveTrainController[2]);
            telemetry.addData("I Movement:", intakePower);
            telemetry.update();




        }
    }
}

