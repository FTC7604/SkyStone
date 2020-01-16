package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DWAIAutonomous;

import static org.firstinspires.ftc.teamcode.DWAIAutonomous.ALLIANCE;
import static org.firstinspires.ftc.teamcode.DWAIAutonomous.PARK_POSITION;
import static org.firstinspires.ftc.teamcode.DWAIAutonomous.SIDE;

@Autonomous(name = "Red Block Horizontal Bridge", group = "Autonomous")
public class RED_BLOCK_HORIZONTAL_BRIDGE extends LinearOpMode {
    private DWAIAutonomous auto;


    @Override
    public void runOpMode() {
        auto = new DWAIAutonomous(DWAIAutonomous.FOUNDATION_ORIENTATION.HORIZONTAL, PARK_POSITION.BRIDGE, SIDE.BLOCK, ALLIANCE.RED, this);
        auto.runOpMode();
    }

}
