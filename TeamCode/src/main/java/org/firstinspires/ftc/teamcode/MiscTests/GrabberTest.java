package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.Robot.*;

import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD;


@TeleOp(name="Grabber Test")
public class GrabberTest extends LinearOpMode{
    private RobotLinearOpMode robot;

    @Override
    public void runOpMode(){
        robot = new RobotLinearOpMode(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.clearAll();

        while(!isStopRequested()){

            if(gamepad1.a){
                while(gamepad1.a){}
                robot.setLeftGrabberPosition(gamepad1.left_stick_y, gamepad1.left_stick_x);

                //start pos: grabber @ 1, servo @ 0
                //grab pos: grabber @ 0.4, servo @ 0
                //float pos: grabber @ 0.4, servo @ 0.1

                telemetry.addData("Grabber position", gamepad1.left_stick_y);
                telemetry.addData("Servo position", gamepad1.left_stick_x);
                telemetry.update();
            }

        }

    }

}
