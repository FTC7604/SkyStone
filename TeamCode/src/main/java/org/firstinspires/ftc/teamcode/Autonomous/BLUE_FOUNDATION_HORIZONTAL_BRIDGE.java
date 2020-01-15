package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DWAIAutonomous;

import static org.firstinspires.ftc.teamcode.DWAIAutonomous.ALLIANCE;
import static org.firstinspires.ftc.teamcode.DWAIAutonomous.PARK_POSITION;
import static org.firstinspires.ftc.teamcode.DWAIAutonomous.PLATFORM_ORIENTATION;
import static org.firstinspires.ftc.teamcode.DWAIAutonomous.SIDE;

@Autonomous(name = "BLUE_FOUNDATION_HORIZONTAL_BRIDGE", group = "Autonomous")
public class BLUE_FOUNDATION_HORIZONTAL_BRIDGE extends LinearOpMode {
    private DWAIAutonomous auto;


    @Override
    public void runOpMode() {
        auto = new DWAIAutonomous(PLATFORM_ORIENTATION.HORIZONTAL, PARK_POSITION.BRIDGE, SIDE.FOUNDATION, ALLIANCE.BLUE, this);
        auto.runOpMode();
    }

}
