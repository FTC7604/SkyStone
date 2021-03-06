package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DWAIAutonomous;

import static org.firstinspires.ftc.teamcode.DWAIAutonomous.ALLIANCE;
import static org.firstinspires.ftc.teamcode.DWAIAutonomous.PARK_POSITION;
import static org.firstinspires.ftc.teamcode.DWAIAutonomous.FOUNDATION_ORIENTATION;
import static org.firstinspires.ftc.teamcode.DWAIAutonomous.SIDE;

@Autonomous(name = "Blue Foundation Vertical Wall", group = "Autonomous")
public class BLUE_FOUNDATION_VERTICAL_WALL extends LinearOpMode {
    private DWAIAutonomous auto;

    @Override
    public void runOpMode() {
        auto = new DWAIAutonomous(FOUNDATION_ORIENTATION.VERTICAL, PARK_POSITION.WALL, SIDE.FOUNDATION, ALLIANCE.BLUE, this);
        auto.runOpMode();
    }

}
