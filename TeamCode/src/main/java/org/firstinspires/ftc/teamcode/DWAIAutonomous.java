package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.*;

import java.util.FormatFlagsConversionMismatchException;

import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.STRAFE;

@TeleOp(name = "Will's Autonomous Prototype", group = "TeleOp")
public class DWAIAutonomous extends LinearOpMode {

    private PropertiesLoader propertiesLoader = new PropertiesLoader("Autonomous");

    private double DRIVETRAIN_DISTANCE_RIGHT_TO_GET_FOUNDATION = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_RIGHT_TO_GET_FOUNDATION");
    private double DRIVETRAIN_DISTANCE_BACKWARD_TO_GET_OFF_WALL = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_BACKWARD_TO_GET_OFF_WALL");

    private boolean PAUSE_RIGHT_TO_GET_FOUNDATION = propertiesLoader.getBooleanProperty("PAUSE_RIGHT_TO_GET_FOUNDATION");
    private boolean PAUSE_BACKWARD_TO_GET_OFF_WALL = propertiesLoader.getBooleanProperty("PAUSE_BACKWARD_TO_GET_OFF_WALL");

    private double DRIVETRAIN_DISTANCE_BACKWARD_TO_GET_FOUNDATION = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_BACKWARD_TO_GET_FOUNDATION");
    private double DRIVETRAIN_DISTANCE_FORWARD_TO_DEPOT = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_FORWARD_TO_DEPOT");

    private boolean PAUSE_BACKWARD_TO_GET_FOUNDATION = propertiesLoader.getBooleanProperty("PAUSE_BACKWARD_TO_GET_FOUNDATION");
    private boolean PAUSE_FORWARD_TO_DEPOT = propertiesLoader.getBooleanProperty("PAUSE_FORWARD_TO_DEPOT");

    private double DRIVETRAIN_DISTANCE_LEFT_TO_CLEAR_FOUNDATION = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_LEFT_TO_CLEAR_FOUNDATION");
    private double DRIVETRAIN_DISTANCE_BACKWARD_TO_MIDDLE_OF_FOUNDATION = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_BACKWARD_TO_MIDDLE_OF_FOUNDATION");

    private boolean PAUSE_LEFT_TO_CLEAR_FOUNDATION = propertiesLoader.getBooleanProperty("PAUSE_LEFT_TO_CLEAR_FOUNDATION");
    private boolean PAUSE_BACKWARD_TO_MIDDLE_OF_FOUNDATION = propertiesLoader.getBooleanProperty("PAUSE_BACKWARD_TO_MIDDLE_OF_FOUNDATION");

    //private boolean PAUSE_LATCH = propertiesLoader.getBooleanProperty("PAUSE_LATCH");
    private long PAUSE_TIME = propertiesLoader.getLongProperty("PAUSE_TIME");

    private boolean PAUSE_FORWARD_TO_TURN = propertiesLoader.getBooleanProperty("PAUSE_FORWARD_TO_TURN");
    private double DRIVETRAIN_DISTANCE_FORWARD_TO_TURN = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_FORWARD_TO_TURN");

    private double PARK_STRAFE_DISTANCE = propertiesLoader.getDoubleProperty("PARK_STRAFE_DISTANCE");

    //private boolean PAUSE_UP_TO_RELEASE_BLOCK = propertiesLoader.getBooleanProperty("PAUSE_UP_TO_RELEASE_BLOCK");
    //private boolean PAUSE_DOWN_TO_RELEASE_BLOCK = propertiesLoader.getBooleanProperty("PAUSE_DOWN_TO_RELEASE_BLOCK");
    //private double ARM_ENCODER_TO_RELEASE_BLOCK = propertiesLoader.getDoubleProperty("ARM_ENCODER_TO_RELEASE_BLOCK");

    private ElapsedTime runtime = new ElapsedTime();

    private RobotLinearOpMode robot;

    @Override
    public void runOpMode() {
        robot = new RobotLinearOpMode(this, COLOR_SENSOR.UNDER);

        robot.setAllMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setAllMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setAllMotorZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.initIMU();

        waitForStart();
        runtime.reset();

        alignToFoundationFromFoundationSide();
        getFoundationCurve();
        park();
    }

    private void alignToFoundationFromFoundationSide(){
        if(PAUSE_BACKWARD_TO_GET_OFF_WALL)print("Moving off of the wall");
        robot.moveByInches(DRIVETRAIN_DISTANCE_BACKWARD_TO_GET_OFF_WALL, FORWARD);

        if(PAUSE_RIGHT_TO_GET_FOUNDATION)print("Moving right to foundation");
        robot.moveByInches(DRIVETRAIN_DISTANCE_RIGHT_TO_GET_FOUNDATION, STRAFE);

        if(PAUSE_BACKWARD_TO_GET_FOUNDATION)print("Moving to build plate");
        robot.moveByInches(DRIVETRAIN_DISTANCE_BACKWARD_TO_GET_FOUNDATION, FORWARD);
    }

    private void latchFoundation(){
        robot.openLatch();

        while(!robot.getFoundationSensorPressed()){
            robot.mecanumPowerDrive(0,-.3,0);
        }

        robot.closeLatch();

        robot.moveByInches(-2, FORWARD, .6);
    }

    private void getFoundationCurve(){
        latchFoundation();

        //forward
        if(PAUSE_FORWARD_TO_TURN)print("Driving forward to the platform");
        robot.moveByInches(DRIVETRAIN_DISTANCE_FORWARD_TO_TURN, FORWARD);

        print("Turning to be forward");
        robot.turnByDegree(90);
        robot.openLatch();
    }

    private void park(){
        //Strafe against wall
        robot.moveByInches(-3, STRAFE);
        robot.moveByInches(PARK_STRAFE_DISTANCE, STRAFE);

        //Deploy function
        robot.setLiftPower(-0.2);
        sleep(2000);
        robot.setArmPower(.2);
        sleep(600);
        robot.setLiftPower(0.2);
        sleep(150);
        robot.setLiftZeroPowerProperty(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.setLiftPower(0);
        robot.setArmPower(-0.2);
        sleep(50);
        robot.setArmZeroPowerProperty(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.setArmPower(0);
        sleep(1000);
        robot.setLiftRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setLiftRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setArmRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setArmRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Forward under bridge
        robot.moveByInches(36, FORWARD);
    }

    private void getFoundationStraight() {
        latchFoundation();

        if(PAUSE_FORWARD_TO_DEPOT)print("Driving forward to the platform");
        robot.moveByInches(DRIVETRAIN_DISTANCE_FORWARD_TO_DEPOT, FORWARD);

        robot.openLatch();

        if(PAUSE_LEFT_TO_CLEAR_FOUNDATION)print("Strafing away from the platform");
        robot.moveByInches(DRIVETRAIN_DISTANCE_LEFT_TO_CLEAR_FOUNDATION, STRAFE);

        if(PAUSE_BACKWARD_TO_MIDDLE_OF_FOUNDATION)print("Moving up parallel with platform");
        robot.moveByInches(DRIVETRAIN_DISTANCE_BACKWARD_TO_MIDDLE_OF_FOUNDATION, FORWARD);

        print("Turning to be forward");
        robot.turnByDegree(85);

    }

    private void print(String printString) {
        telemetry.addLine(printString);
        telemetry.update();

        pause(PAUSE_TIME);
    }

    private void pause(double time){
        double currentTime = runtime.milliseconds();

        while(opModeIsActive() && runtime.milliseconds() - currentTime < time){
            robot.stopAllMotors();
        }
    }

}
