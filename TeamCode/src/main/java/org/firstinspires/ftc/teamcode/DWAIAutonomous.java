package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.*;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.*;

public class DWAIAutonomous {

    private PropertiesLoader propertiesLoader = new PropertiesLoader("Autonomous");

    private double DRIVETRAIN_DISTANCE_RIGHT_TO_GET_FOUNDATION = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_RIGHT_TO_GET_FOUNDATION");
    private double DRIVETRAIN_DISTANCE_BACKWARD_TO_GET_OFF_WALL = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_BACKWARD_TO_GET_OFF_WALL");
    private double DRIVETRAIN_DISTANCE_BACKWARD_TO_GET_FOUNDATION = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_BACKWARD_TO_GET_FOUNDATION");
    private double DRIVETRAIN_DISTANCE_FORWARD_TO_DEPOT = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_FORWARD_TO_DEPOT");
    private double DRIVETRAIN_DISTANCE_LEFT_TO_CLEAR_FOUNDATION = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_LEFT_TO_CLEAR_FOUNDATION");
    private double DRIVETRAIN_DISTANCE_BACKWARD_TO_MIDDLE_OF_FOUNDATION = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_BACKWARD_TO_MIDDLE_OF_FOUNDATION");
    private double DRIVETRAIN_DISTANCE_FORWARD_TO_TURN = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_FORWARD_TO_TURN");
    private double DRIVETRAIN_DISTANCE_BACKWARD_TO_SLAM_FOUNDATION_REALLY_REALLY_HARD = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_BACKWARD_TO_SLAM_FOUNDATION_REALLY_REALLY_HARD");

    private double HWALL_PARK_STRAFE_DISTANCE = propertiesLoader.getDoubleProperty("HWALL_PARK_STRAFE_DISTANCE");
    private double HBRIDGE_PARK_STRAFE_DISTANCE = propertiesLoader.getDoubleProperty("HBRIDGE_PARK_STRAFE_DISTANCE");
    private double VWALL_PARK_STRAFE_DISTANCE = propertiesLoader.getDoubleProperty("VWALL_PARK_STRAFE_DISTANCE");
    private double VBRIDGE_PARK_STRAFE_DISTANCE = propertiesLoader.getDoubleProperty("VBRIDGE_PARK_STRAFE_DISTANCE");
    private boolean PAUSE_STEPS = propertiesLoader.getBooleanProperty("PAUSE_STEPS");
    private double STRAFE_MAX_POWER = propertiesLoader.getDoubleProperty("STRAFE_MAX_POWER");
    private double MOVE_MAX_POWER = propertiesLoader.getDoubleProperty("MOVE_MAX_POWER");

    private double OPEN_LATCH_POSITION = propertiesLoader.getDoubleProperty("OPEN_LATCH_POSITION");
    private double CLOSE_LATCH_POSITION = propertiesLoader.getDoubleProperty("CLOSE_LATCH_POSITION");

    private double horizontalTurnDegree = 90;
    private double fiddleDistance = -3;

    private ElapsedTime runtime = new ElapsedTime();

    private RobotLinearOpMode robot;

    private PLATFORM_ORIENTATION platformOrientation;
    private PARK_POSITION parkPosition;
    private SIDE side;
    private ALLIANCE alliance;
    private LinearOpMode opMode;

    public DWAIAutonomous(PLATFORM_ORIENTATION platformOrientation, PARK_POSITION parkPosition, SIDE side, ALLIANCE alliance, LinearOpMode opMode){
        this.platformOrientation = platformOrientation;
        this.parkPosition = parkPosition;
        this.side = side;
        this.alliance = alliance;
        this.opMode = opMode;
    }

    public void runOpMode(){
        robot = new RobotLinearOpMode(opMode);

        //MAYBE PUT INTO CONSTRUCTOR
        robot.setAllMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setAllMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setAllMotorZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.initIMU();
        setupVariables();

        opMode.waitForStart();
        runtime.reset();

        if(side == SIDE.FOUNDATION) {
            moveToFoundation();
            latchFoundation();

            if(platformOrientation == PLATFORM_ORIENTATION.HORIZONTAL) {
                placeHorizontal();
                parkHorizontal();
            } else{
                placeVertical();
                parkVertical();
            }

        }

    }

    private void setupVariables(){
        //Necessary since all variables tuned to blue side

        if(alliance == ALLIANCE.RED && side == SIDE.FOUNDATION){
            DRIVETRAIN_DISTANCE_RIGHT_TO_GET_FOUNDATION *= -1;
            horizontalTurnDegree *= -1;
            fiddleDistance *= -1;
            HWALL_PARK_STRAFE_DISTANCE *= -1;
            HBRIDGE_PARK_STRAFE_DISTANCE *= -1;
        }

    }

    private void moveToFoundation(){
        print("Moving towards foundation");
        robot.moveByInches(DRIVETRAIN_DISTANCE_BACKWARD_TO_GET_OFF_WALL, FORWARD);
        robot.moveByInches(DRIVETRAIN_DISTANCE_RIGHT_TO_GET_FOUNDATION, STRAFE);
        robot.moveByInchesMaxPower(DRIVETRAIN_DISTANCE_BACKWARD_TO_GET_FOUNDATION, FORWARD, MOVE_MAX_POWER);
    }

    private void latchFoundation(){
        print("Latching foundation");
        robot.setLatchPosition(OPEN_LATCH_POSITION);

        while(!robot.getFoundationSensorPressed()){
            robot.mecanumPowerDrive(0,-.3,0);
        }

        robot.setLatchPosition(CLOSE_LATCH_POSITION);
        robot.moveByInchesMaxPower(-2, FORWARD, .6);
    }

    private void placeHorizontal(){
        print("Driving forwards");
        robot.moveByInches(DRIVETRAIN_DISTANCE_FORWARD_TO_TURN, FORWARD);

        print("Turning to be forward");

        robot.turnToDegree(horizontalTurnDegree);
        robot.setLatchPosition(OPEN_LATCH_POSITION);
    }

    private void parkHorizontal(){
        print("Fiddling with latch");
        robot.moveByInches(fiddleDistance, STRAFE);

        if(parkPosition == PARK_POSITION.WALL) {
            print("Strafing against wall");
            robot.moveByInches(HWALL_PARK_STRAFE_DISTANCE, STRAFE);
        } else{
            print("Strafing towards bridge");
            robot.moveByInches(HBRIDGE_PARK_STRAFE_DISTANCE, STRAFE);
        }

        print("Deploying intake");
        robot.deploy();

        print("Moving under bridge");
        robot.moveByInches(36, FORWARD);
        //rough movement, likely replace with color sensor line code
    }

    private void placeVertical(){
        print("Driving forwards");
        robot.moveByInches(DRIVETRAIN_DISTANCE_FORWARD_TO_DEPOT, FORWARD);

        print("Opening latch");
        robot.setLatchPosition(OPEN_LATCH_POSITION);

        print("Strafing away from the platform");
        robot.moveByInchesMaxPower(DRIVETRAIN_DISTANCE_LEFT_TO_CLEAR_FOUNDATION, STRAFE, STRAFE_MAX_POWER);

        print("Moving up parallel with platform");
        robot.moveByInches(DRIVETRAIN_DISTANCE_BACKWARD_TO_MIDDLE_OF_FOUNDATION, FORWARD);

        print("Turning to be forward");

        robot.turnToDegree(90);

        print("Slamming foundation against the wall really really hard");
        robot.moveByInches(DRIVETRAIN_DISTANCE_BACKWARD_TO_SLAM_FOUNDATION_REALLY_REALLY_HARD, FORWARD);
    }

    private void parkVertical(){

        if(parkPosition == PARK_POSITION.WALL){
            print("Strafing against wall");
            robot.moveByInches(VWALL_PARK_STRAFE_DISTANCE, STRAFE);
        } else{
            print("Strafing against bridge");
            robot.moveByInches(VBRIDGE_PARK_STRAFE_DISTANCE, STRAFE);
        }

        print("Moving under bridge");
        robot.moveByInches(22, FORWARD);

        //currently just exact copy of horizontal parking method
        /*print("Fiddling with latch");
        robot.moveByInches(fiddleDistance, STRAFE);

        if(parkPosition == PARK_POSITION.WALL) {
            print("Strafing against wall");
            robot.moveByInches(WALL_PARK_STRAFE_DISTANCE, STRAFE);
        } else{
            print("Strafing towards bridge");
            robot.moveByInches(BRIDGE_PARK_STRAFE_DISTANCE, STRAFE);
        }

        print("Deploying intake");
        robot.deploy();

        print("Moving under bridge");
        robot.moveByInches(36, FORWARD);*/
    }

    private void print(String printString) {
        opMode.telemetry.addLine(printString);
        opMode.telemetry.update();

        if (PAUSE_STEPS){
            robot.stopAllMotors();
            //Ensures button is pressed and released before continuing
            while (opMode.opModeIsActive() && !opMode.gamepad1.a){}
            while (opMode.opModeIsActive() && opMode.gamepad1.a){}
        }

    }

    public enum PLATFORM_ORIENTATION {
        HORIZONTAL,
        VERTICAL
    }

    public enum PARK_POSITION {
        WALL,
        BRIDGE
    }

    public enum ALLIANCE {
        RED,
        BLUE
    }

    public enum SIDE {
        BLOCK,
        FOUNDATION
    }

}
