package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.Robot.*;

import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD;


@TeleOp(name="Compensation Test")
@Disabled
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
                robot.compensatingMoveByInchesFast(24, FORWARD, 0);
            }

        }

    }

}
