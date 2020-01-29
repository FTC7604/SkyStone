package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.Robot.*;

import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD;


@TeleOp(name="Grabber Test")
public class GrabberTest extends LinearOpMode{
    private RobotLinearOpMode robot;

    @Override
    public void runOpMode(){
        double pos1 = 0;
        double pos2 = 0;

        robot = new RobotLinearOpMode(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.clearAll();

        while(!isStopRequested()){

            if(gamepad1.a){
                while(gamepad1.a){}
                pos1 -= 0.1;
            }

            if(gamepad1.y){
                while(gamepad1.y){}
                pos1 += 0.1;
            }

            if(gamepad1.x){
                while(gamepad1.x){}
                pos2 -= 0.1;
            }

            if(gamepad1.b){
                while(gamepad1.b){}
                pos2 += 0.1;
            }

            robot.setLeftGrabberPosition(pos1, pos2);

            //left
            //float pos: grabber @ 0.45, servo @ 0.6 (stowed)
            //grab pos: grabber @ 0.45, servo @ 0
            //start pos: grabber @ 1, servo @ 0
            //default pos: grabber @ 0.3, servo @ 0.8

            //right
            //float pos: grabber @ 0.7, servo @ 0.3 (stowed)
            //grab pos: grabber @ 0.7, servo @ 0.8
            //start pos: grabber @ 0, servo @ 0.8
            //default pos: grabber @ 0.85, servo @ -0.1

            telemetry.addData("Grabber position", pos1);
            telemetry.addData("Servo position", pos2);
            telemetry.update();
        }

    }

}
