package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

import static org.firstinspires.ftc.teamcode.DWAIAutonomous.*;

@TeleOp(name = "AutoTest", group = "TeleOp")
public class AutoTest extends LinearOpMode {
    private DWAIAutonomous auto;

    @Override
    public void runOpMode(){
        auto = new DWAIAutonomous(PLATFORM_ORIENTATION.HORIZONTAL, PARK_POSITION.BRIDGE, ALLIANCE.BLUE, this);
        auto.runOpMode();
    }

}
