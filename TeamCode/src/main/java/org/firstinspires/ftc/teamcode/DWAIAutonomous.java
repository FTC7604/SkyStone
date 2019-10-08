package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.Robot.*;
import org.firstinspires.ftc.teamcode.Control.*;
import org.firstinspires.ftc.teamcode.LED.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.*;

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

        everythingbluefountationside();

        //FULL ASS AUTONOMOUS
    }

    enum AUTO_PATHS {
        BLUE_FOUNDATIONSIDE_PARK,
        BLUE_FOUNDATIONSIDE_,

        BLUE_STONESIDE_,

        RED_FOUNDATIONSIDE_,

        RED_STONESIDE_,

    }

    void everythingbluefountationside(){
//        //intake facing away from the field
//        //strafe right to align with build platform
//        robot.moveByInches(24, X);
//        //backwards unit hit platform
//        robot.moveByInches(-48, Y);
//        //latch once build platform is hit while moving
//        robot.closeLatch();
//
//
//
//        //2 options:
//
//
//
//        //forward until hit the wall
//        robot.moveByInches(50, Y);
//        //unlatch the build plate
//        robot.openLatch();
//        //strafe left out of the entrappment
//        robot.moveByInches(-36, X);
//        //backward til aligned with platform
//        robot.moveByInches(50, Y);
//        //strafe left unit enough room to turn
//        //turn 90 deg to the left
//        robot.turnByDegree(90);
//        //backward until platform hits the wall
//        robot.moveByInches(-6, Y);



        //or



        //turn right by x degrees
        //forward y inches
        //turn left by x degree
        //the trig works out such that y*sin(x) = 4 inches and y*cos(x) ~< 3 feet, so that on the build plate is properly aligned
        //forward until the wall is hit
        //strafe left out of the entrappment
        //bkacward to align with build plate
        ///turn 90 deg to the left



        //forward until start of blocks
        //slow down but forward until a block is seen
        //backward enough so I won't knock over the stystone when strafing
        //strafe left to align intake with skytone
        //move forward and intake until one block has been intook




        //2 paths:



        //grip with the grabber
        //lift the arm enough so that it is not touching the ground or as much as is possible
        //




        //or




        //continuously intake so that the brick never leaves




        //backward to move away from the other bricks
        //strafe right until aligned with the build plate
        //backward until underneath the line
        //sense the line with under the bridge or know from encoder values
        //continue going backward but lift the arm
        //stop in front of the build plate
        //when the arm is completely lowered
        //drop the block and close the grabber



        //2 paths:




        //don't lift the arm and drive under the bridge
        //sense when you drive under the bridge
        //start to lift the arm once you go under the bridge and continue to do so




        //or




        //start putting down the lift immediately
        //sense where the skybride is and stop right before it
        //wait for the raminder of the lift to go down




        //drive forward the necessary amount to collide into the next set of stones
        //strafe to the left the correct amount to do so

    }

}
