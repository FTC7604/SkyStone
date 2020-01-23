package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.IO.RuntimeLogger;
import org.firstinspires.ftc.teamcode.Robot.*;

@TeleOp(name="Turn Test")
public class TurnTest extends LinearOpMode{
    private RobotLinearOpMode robot;

    @Override
    public void runOpMode(){
        robot = new RobotLinearOpMode(this);

        double rot = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.clearAll();

        while(!isStopRequested()){

            if(gamepad1.a){
                while(gamepad1.a){}

                robot.turnToDegreeFast(rot + 90);
                rot += 90;

                if(rot > 360){
                    rot = 90;
                }

            }

        }

    }

}
