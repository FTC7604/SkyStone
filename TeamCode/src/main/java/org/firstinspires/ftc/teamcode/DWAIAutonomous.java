package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.*;

import java.util.FormatFlagsConversionMismatchException;

import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.STRAFE;

@TeleOp(name = "Will's Autonomous Prototype", group = "TeleOp")
@Disabled
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

    private boolean PAUSE_LATCH = propertiesLoader.getBooleanProperty("PAUSE_LATCH");
    private long PAUSE_TIME = propertiesLoader.getLongProperty("PAUSE_TIME");

    private boolean PAUSE_FORWARD_TO_TURN = propertiesLoader.getBooleanProperty("PAUSE_FORWARD_TO_TURN");
    private double DRIVETRAIN_DISTANCE_FORWARD_TO_TURN = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_FORWARD_TO_TURN");

    private boolean PAUSE_UP_TO_RELEASE_BLOCK = propertiesLoader.getBooleanProperty("PAUSE_UP_TO_RELEASE_BLOCK");
    private boolean PAUSE_DOWN_TO_RELEASE_BLOCK = propertiesLoader.getBooleanProperty("PAUSE_DOWN_TO_RELEASE_BLOCK");
    private double ARM_ENCODER_TO_RELEASE_BLOCK = propertiesLoader.getDoubleProperty("ARM_ENCODER_TO_RELEASE_BLOCK");


//    private double ARM_LIFT_UP_ENCODER_DISTANCE = propertiesLoader.getDoubleProperty("ARM_LIFT_UP_ENCODER_DISTANCE");
//    private double ARM_LIFT_DOWN_ENCODER_DISTANCE = propertiesLoader.getDoubleProperty("ARM_LIFT_DOWN_ENCODER_DISTANCE");
//
//    private double MAX_BOT_MOVEMENT_POWER_WHEN_INTAKING = propertiesLoader.getDoubleProperty("MAX_BOT_MOVEMENT_POWER_WHEN_INTAKING");
//    private double MAX_BOT_INTAKE_POWER_WHEN_INTAKING = propertiesLoader.getDoubleProperty("MAX_BOT_INTAKE_POWER_WHEN_INTAKING");
//    private double EXTRA_DECELERATION_ENCODER_TICKS_WHEN_INTAKING = propertiesLoader.getDoubleProperty("EXTRA_DECELERATION_ENCODER_TICKS_WHEN_INTAKING");
//
//    private double ARM_HEIGHT_WHEN_INTAKING = propertiesLoader.getDoubleProperty("ARM_HEIGHT_WHEN_INTAKING");
//
//    private double STRAFE_TO_STONE = propertiesLoader.getDoubleProperty("STRAFE_TO_STONE");
//    private double BACKWARD_TO_STONE = propertiesLoader.getDoubleProperty("BACKWARD_TO_STONE");
//

//
//    //private double DISTANCE_TO_PUSH_BUILD_PLATE_INTO_WALL = propertiesLoader.getDoubleProperty("DISTANCE_TO_PUSH_BUILD_PLATE_INTO_WALL");
//    //private double DISTANCE_TO_GET_STONE = propertiesLoader.getDoubleProperty("DISTANCE_TO_GET_STONE");


    private ElapsedTime runtime = new ElapsedTime();

    private RobotLinearOpMode robot;
    //private int whatever = propertiesLoader.getIntegerProperty("whatever");



    @Override
    public void runOpMode() {

        robot = new RobotLinearOpMode(this, COLOR_SENSOR.NONE);

        robot.setAllMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setAllMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setAllMotorZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.initIMU();


        waitForStart();
        runtime.reset();

        alignToFoundationFromFoundationSide();
        getFoundationCurve();
        //getFoundationStraight();

        //robot.moveByInchesWill(36,FORWARD);

//        robot.openLatch();
//        while (!robot.foundationIsNear()){
//            robot.mecanumPowerDrive(0,-1,0);
//        }
//        robot.closeLatch();




//        robot.moveByInches(DISTANCE_TO_GET_STONE, FORWARD, true);
//        print("It moved forward");
//
//        //getTheSecondOrThirdBlock(1);
//
//        robot.moveByInches(-DISTANCE_TO_GET_STONE - DISTANCE_TO_PUSH_BUILD_PLATE_INTO_WALL, FORWARD, true);
//        print("It moved forward");
//
//        dropOffBlock();
//
//        robot.moveByInches(DISTANCE_TO_GET_STONE + DISTANCE_TO_PUSH_BUILD_PLATE_INTO_WALL, FORWARD, true);
//        print("It moved forward");
//
//        //getTheFourthFifthOrSixthBlock(4);
//
//        robot.moveByInches(-DISTANCE_TO_GET_STONE - DISTANCE_TO_PUSH_BUILD_PLATE_INTO_WALL, FORWARD, true);
//        print("It moved forward");

        //dropOffBlock();

    }


//
//    private void getTheSecondOrThirdBlock(int stoneNumber){
//        robot.moveByInches(8 * stoneNumber,FORWARD,true);
//        print("moving to Stone");
//
//        robot.moveByInches(STRAFE_TO_STONE, STRAFE, true);
//        print("It moved left");
//
//        robot.moveByInches(BACKWARD_TO_STONE, FORWARD, true);
//        print("It moved backward");
//
//        openIntake();
//
//        pickUpBlock();
//
//        robot.setIntakePower(-1);
//        print("It removed the block from the intake");
//
//
//        robot.moveByInches(-8 * stoneNumber,FORWARD,true);
//        print("moving to Stone");
//
//
//        robot.moveByInches(-STRAFE_TO_STONE, STRAFE, true);
//        print("It moved left, but opposite");
//    }
//    private void getTheFourthFifthOrSixthBlock(int stoneNumber){
//        robot.moveByInches(8 * stoneNumber,FORWARD,true);
//        print("moving to Stone");
//
//        robot.moveByInches(STRAFE_TO_STONE, STRAFE, true);
//        print("It moved left");
//
//        robot.moveByInches(BACKWARD_TO_STONE, FORWARD, true);
//        print("It moved backward");
//
//        pickUpBlock();
//
//        robot.setIntakePower(-1);
//        print("It removed the block from the intake");
//
//
//        robot.moveByInches(-8 * stoneNumber,FORWARD,true);
//        print("moving to Stone");
//
//
//        robot.moveByInches(-STRAFE_TO_STONE, STRAFE, true);
//        print("It moved left, but opposite");
//    }
//
//    private void pickUpBlock(){
//        robot.openGrabber();
//        print("Opening the grabber");
//
//        robot.moveToStone(MAX_BOT_MOVEMENT_POWER_WHEN_INTAKING,MAX_BOT_INTAKE_POWER_WHEN_INTAKING,EXTRA_DECELERATION_ENCODER_TICKS_WHEN_INTAKING);
//        print("Moving to Stone");
//
//        robot.closeGrabber();
//
//        robot.blockHasLeftIntake();
//        print("Closing the grabber and stating that the block has left");
//
//        robot.moveArmByEncoder(ARM_HEIGHT_WHEN_INTAKING);
//        print("Raise the arm by 1000");
//
//        robot.setIntakePower(-MAX_BOT_INTAKE_POWER_WHEN_INTAKING);
//        print("It removed the block from the intake");
//    }
//
//    private void openIntake() {
//        robot.moveArmByEncoder(ARM_LIFT_UP_ENCODER_DISTANCE);
//        print("Lifting the arm up");
//
//        robot.moveArmByEncoder(ARM_LIFT_DOWN_ENCODER_DISTANCE);
//        print("Putting the arm down");
//    }

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
            robot.mecanumPowerDrive(0,-.6,0);
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
    private void getFoundationStraight() {
        latchFoundation();

        if(PAUSE_FORWARD_TO_DEPOT)print("Driving forward to the platform");
        robot.moveByInches(DRIVETRAIN_DISTANCE_FORWARD_TO_DEPOT, FORWARD);

        robot.openLatch();

        if(PAUSE_LEFT_TO_CLEAR_FOUNDATION)print("Strafing away from the platform");
        robot.moveByInches(DRIVETRAIN_DISTANCE_LEFT_TO_CLEAR_FOUNDATION, STRAFE);

        if(PAUSE_BACKWARD_TO_MIDDLE_OF_FOUNDATION)print("Moving up parrelel with platform");
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
