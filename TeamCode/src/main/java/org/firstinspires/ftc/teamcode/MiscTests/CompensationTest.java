package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.Robot.*;

import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD;


@TeleOp(name="Movement Test")
public class CompensationTest extends LinearOpMode{
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
                robot.moveByInchesFast(-72, FORWARD);
            } else if(gamepad1.b){
                while(gamepad1.b){}
                robot.compensatingMoveByInchesFast(-72, FORWARD, 0);
            }

        }

    }

}
