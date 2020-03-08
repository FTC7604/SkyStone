package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DWAIAutonomous;

@TeleOp(name = "Trajectory Tester RED")
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        DWAIAutonomous auto = new DWAIAutonomous(
                DWAIAutonomous.FOUNDATION_ORIENTATION.VERTICAL,
                DWAIAutonomous.PARK_POSITION.BRIDGE,
                DWAIAutonomous.SIDE.BLOCK,
                DWAIAutonomous.ALLIANCE.RED,
                this);

        auto.runOpMode();
    }
}
