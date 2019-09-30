package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.Robot.*;

@Autonomous(name = "Autonomous Prototype", group = "Autonomous")
public class DWAIAutonomous extends LinearOpMode {
    private PropertiesLoader propertiesLoader = new PropertiesLoader("Autonomous");
    private RobotLinearOpMode robot;

    //private int whatever = propertiesLoader.getIntegerProperty("whatever");

    public void initializeAutonomous(){
        robot = new RobotLinearOpMode(this);
    }

    @Override
    public void runOpMode(){
        initializeAutonomous();
        waitForStart();

        //FULL ASS AUTONOMOUS
    }

}
