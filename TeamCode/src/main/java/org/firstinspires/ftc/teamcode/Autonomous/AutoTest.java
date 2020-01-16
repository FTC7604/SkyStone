package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.DWAIAutonomous;
import org.firstinspires.ftc.teamcode.IO.DWAIAutonomousPropertiesLoader;

import static org.firstinspires.ftc.teamcode.DWAIAutonomous.*;

@TeleOp(name = "AutoTest", group = "TeleOp")
public class AutoTest extends LinearOpMode {
    private DWAIAutonomous auto;
    private DWAIAutonomousPropertiesLoader autonomousPropertiesLoader = new DWAIAutonomousPropertiesLoader();

    private FOUNDATION_ORIENTATION platformOrientation = autonomousPropertiesLoader.getPlatformOrientationProperty("PLATFORM_ORIENTATION");
    private PARK_POSITION parkPosition = autonomousPropertiesLoader.getParkPositionProperty("PARK_POSITION");
    private SIDE side = autonomousPropertiesLoader.getSideProperty("SIDE");
    private ALLIANCE alliance = autonomousPropertiesLoader.getAllianceProperty("ALLIANCE");

    @Override
    public void runOpMode(){
        auto = new DWAIAutonomous(platformOrientation, parkPosition, side, alliance, this);
        auto.runOpMode();
    }

}
