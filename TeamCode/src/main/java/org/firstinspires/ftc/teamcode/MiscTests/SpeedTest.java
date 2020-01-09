package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Toggle;
import org.firstinspires.ftc.teamcode.Robot.*;

/**  TESTS MAX POWER DRIVE  */

@TeleOp(name = "I am Speed", group = "Linear Opmode")
@Disabled
public class SpeedTest extends LinearOpMode {

    RobotLinearOpMode robotLinearOpMode;

    /*
    0 - forward movement
    1 - backward movement
    2 - right movement
    3 - left movement
    4 - right turn
    5 - left turn
     */
    Toggle[] directions = new Toggle[6];



    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        for(int i = 0; i < directions.length; i++){
            directions[i] = new Toggle(false);
        }

        robotLinearOpMode = new RobotLinearOpMode(this, COLOR_SENSOR.NONE);

        robotLinearOpMode.setDriveTrainZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);
        robotLinearOpMode.setDriveTrainRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robotLinearOpMode.setLiftZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //this is when the robot receives the 'play' command
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            robotLinearOpMode.mecanumPowerDrive(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x
            );

            robotLinearOpMode.setLiftPower(-gamepad2.left_stick_y);


        }
    }
}
