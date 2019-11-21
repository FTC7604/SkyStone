package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Toggle;
import org.firstinspires.ftc.teamcode.Robot.*;

@TeleOp(name = "I am Speed", group = "Linear Opmode")

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

        //robotLinearOpMode.setDriveTrainRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotLinearOpMode.setDriveTrainRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //this is when the robot receives the 'play' command
        waitForStart();
        runtime.reset();

        int x = 0;
        int y = 0;
        int r = 0;

        while (opModeIsActive()) {

            directions[0].update(gamepad1.dpad_up);
            directions[1].update(gamepad1.dpad_down);
            directions[2].update(gamepad1.dpad_right);
            directions[3].update(gamepad1.dpad_left);

            directions[4].update(gamepad1.x);
            directions[5].update(gamepad1.b);

            if(!directions[0].get() && !directions[1].get())x = 0;
            if(directions[0].get() && !directions[1].get())x = 1;
            if(!directions[0].get() && directions[1].get())x = -1;
            if(directions[0].get() && directions[1].get())x = 0;

            if(!directions[2].get() && !directions[3].get())y = 0;
            if(directions[2].get() && !directions[3].get())y = 1;
            if(!directions[2].get() && directions[3].get())y = -1;
            if(directions[2].get() && directions[3].get())y = 0;

            if(!directions[4].get() && !directions[5].get())r = 0;
            if(directions[4].get() && !directions[5].get())r = 1;
            if(!directions[4].get() && directions[5].get())r = -1;
            if(directions[4].get() && directions[5].get())r = 0;

            robotLinearOpMode.mecanumPowerDrive(y,x,r);

        }



    }
}
