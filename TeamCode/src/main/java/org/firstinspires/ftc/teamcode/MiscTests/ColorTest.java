package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.Robot.*;
import org.firstinspires.ftc.teamcode.*;

@TeleOp(name="Color Test")
@Disabled
public class ColorTest extends LinearOpMode{
    private RobotLinearOpMode robot;
    private RuntimeLogger logger = new RuntimeLogger("colorVals");

    @Override
    public void runOpMode(){
        robot = new RobotLinearOpMode(this);

        double[] colors;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.clearAll();

        while(!isStopRequested()){
            colors = robot.getColors();
            telemetry.addLine("R: " + colors[0] + " G: " + colors[1] + " B: " + colors[2]);
            telemetry.update();
            logger.write("R: " + colors[0] + " G: " + colors[1] + " B: " + colors[2]);
        }

    }

}
