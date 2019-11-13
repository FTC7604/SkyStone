package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode;

import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.STRAFE;

@TeleOp(name = "Will's Autonomous Prototype", group = "TeleOp")
public class DWAIAutonomous extends LinearOpMode {

    private PropertiesLoader propertiesLoader = new PropertiesLoader("Autonomous");

    private double DISTANCE_RIGHT_TO_GET_FOUNDATION = propertiesLoader.getDoubleProperty("DISTANCE_RIGHT_TO_GET_FOUNDATION");
    private double DISTANCE_BACKWARD_TO_GET_OFF_WALL = propertiesLoader.getDoubleProperty("DISTANCE_BACKWARD_TO_GET_OFF_WALL");

    private double DISTANCE_BACKWARD_TO_GET_FOUNDATION = propertiesLoader.getDoubleProperty("DISTANCE_BACKWARD_TO_GET_FOUNDATION");
    private double DISTANCE_BACKWARD_TO_PUSH_FOUNDATION_FOR_LATCH = propertiesLoader.getDoubleProperty("DISTANCE_BACKWARD_TO_PUSH_FOUNDATION_FOR_LATCH");

    private double DISTANCE_FORWARD_TO_DEPOT = propertiesLoader.getDoubleProperty("DISTANCE_FORWARD_TO_DEPOT");

    private double DISTANCE_LEFT_TO_CLEAR_FOUNDATION = propertiesLoader.getDoubleProperty("DISTANCE_LEFT_TO_CLEAR_FOUNDATION");
    private double DISTANCE_BACKWARD_TO_MIDDLE_OF_FOUNDATION = propertiesLoader.getDoubleProperty("DISTANCE_BACKWARD_TO_MIDDLE_OF_FOUNDATION");

    private long PAUSE_TIME = propertiesLoader.getLongProperty("PAUSE_TIME");

    private boolean MOVE_BY_INCHES_FAST = propertiesLoader.getBooleanProperty("MOVE_BY_INCHES_FAST");

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
//    private double ENCODER_TO_DROP_OFF_BLOCK = propertiesLoader.getDoubleProperty("ENCODER_TO_DROP_OFF_BLOCK");
//
//    //private double DISTANCE_TO_PUSH_BUILD_PLATE_INTO_WALL = propertiesLoader.getDoubleProperty("DISTANCE_TO_PUSH_BUILD_PLATE_INTO_WALL");
//    //private double DISTANCE_TO_GET_STONE = propertiesLoader.getDoubleProperty("DISTANCE_TO_GET_STONE");


    private ElapsedTime runtime = new ElapsedTime();

    private RobotLinearOpMode robot;
    //private int whatever = propertiesLoader.getIntegerProperty("whatever");



    @Override
    public void runOpMode() {

        robot = new RobotLinearOpMode(this);

        robot.setAllMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setAllMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setAllMotorZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.initIMU();


        waitForStart();
        runtime.reset();

        alignToFoundationFromFoundationSide();
        getFoundation();


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


//    private void dropOffBlock() {
//        robot.moveArmByEncoder(ENCODER_TO_DROP_OFF_BLOCK);
//
//        robot.openGrabber();
//
//        robot.moveArmByEncoder(-ENCODER_TO_DROP_OFF_BLOCK);
//    }
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

    void alignToFoundationFromFoundationSide(){
        print("Moving off of the wall");
        robot.moveByInches(DISTANCE_BACKWARD_TO_GET_OFF_WALL, FORWARD);

        print("Moving right to foundation");
        robot.moveByInches(DISTANCE_RIGHT_TO_GET_FOUNDATION, STRAFE);
    }

    void getFoundation() {

        print("Opening Latch");
        robot.openLatch();

        print("Moving to build plate");
        robot.moveByInches(DISTANCE_BACKWARD_TO_GET_FOUNDATION, FORWARD);

        print("Moving and latching build plate");
        robot.moveToLatch(DISTANCE_BACKWARD_TO_PUSH_FOUNDATION_FOR_LATCH);

        print("Dragging the build plate to the wall");
        robot.moveByInches(DISTANCE_FORWARD_TO_DEPOT, FORWARD);

        print("Opening Latch");
        robot.openLatch();

        print("Strafing away from the platform");
        robot.moveByInches(DISTANCE_LEFT_TO_CLEAR_FOUNDATION, STRAFE);

        print("Moving up parrelel with platform");
        robot.moveByInches(DISTANCE_BACKWARD_TO_MIDDLE_OF_FOUNDATION, FORWARD);

        print("Strafing the platform into the wall");
        robot.turnByDegree(85);


    }

    void print(String printString) {
        telemetry.addLine(printString);
        telemetry.update();

        pause(PAUSE_TIME);
    }

    void pause(double time){
        double currentTime = runtime.milliseconds();

        while(opModeIsActive() && runtime.milliseconds() - currentTime < time){
            robot.stopAllMotors();
        }
    }

    void everythingbluefountationside() {
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


    enum AUTO_PATHS {
        BLUE_FOUNDATIONSIDE_PARK,
        BLUE_FOUNDATIONSIDE_,

        BLUE_STONESIDE_,

        RED_FOUNDATIONSIDE_,

        RED_STONESIDE_,

    }

}
