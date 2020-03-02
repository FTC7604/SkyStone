/*package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.IO.PropertiesLoader;
import org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode;
import org.firstinspires.ftc.teamcode.Robot.StageSwitchingPipeline;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.GRABBER_POSITION.*;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.STRAFE;


/**
 * IMPORTANT NOTES
 * FOR BLOCK AUTO: VERTICAL = ONE-BLOCK, HORIZONTAL = TWO-BLOCK AT THIS POINT IN TIME
 * LIKELY WILL BE CHANGED BACK FOR FUTURE TOURNAMENTS
 * MUST CHANGE TO + FIGURE OUT ODOMETRY BECAUSE FIELD TILE FRICTION CHANGES
 * WOULD BE VERY NICE TO HAVE A SERVO CLAW ON THE SIDE (QUICKER SAMPLING)
 * REMEMBER TO MAKE A BRIDGE QUICK_PARK AUTO (park is only against the wall right now)
 * REMEMBER TO RECALIBRATE AUTOS (again), POSSIBLY INCLUDE 'friction constant' IN PROPERTIES
 * <p>
 * FIELD SETUP:
 * ALWAYS CHECK ENCODER WIRES AND SUCH!
 * ALWAYS CHECK CAMERA IS NOT BLOCKED BY FOUNDATION CLAMP CLAW!
 * ALWAYS CHECK STRINGS ARE ON PULLEYS!
 * ALWAYS CHECK FOUNDATION CLAMP CLAWS ARE NOT BENT!
 * ALWAYS CHECK LIFT IS CLIPPED ONTO INTAKE!
 * ALWAYS CHECK GAME CONTROLLERS BOTH FUNCTIONING!!!!
 * FOR BLOCK-SIDE: PLACE SIDESKIRT ALIGNED ALONG TILE LINE CLOSE TO DEPOT
 * FOR FOUNDATION-SIDE: PLACE SIDESKIRT ALIGNED ALONG LINE CLOSE TO BRIDGE
 */

/*public class DWAIAutonomous {
    private PropertiesLoader propertiesLoader = new PropertiesLoader("Autonomous");
    private boolean PAUSE_STEPS = propertiesLoader.getBooleanProperty("PAUSE_STEPS");
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
    private double OPEN_LATCH_SERVO_POSITION = propertiesLoader.getDoubleProperty("OPEN_LATCH_SERVO_POSITION");
    private double CLOSE_LATCH_SERVO_POSITION = propertiesLoader.getDoubleProperty("CLOSE_LATCH_SERVO_POSITION");
    private double BLOCK_FORWARD_OFF_WALL_TO_BLOCK = propertiesLoader.getDoubleProperty("BLOCK_FORWARD_OFF_WALL_TO_BLOCK");
    private double BLOCK_ONE_FORWARD_TO_BLOCK = propertiesLoader.getDoubleProperty("BLOCK_ONE_FORWARD_TO_BLOCK");
    private double BLOCK_TWO_FORWARD_TO_BLOCK = propertiesLoader.getDoubleProperty("BLOCK_TWO_FORWARD_TO_BLOCK");
    private double BLOCK_THREE_FORWARD_TO_BLOCK = propertiesLoader.getDoubleProperty("BLOCK_THREE_FORWARD_TO_BLOCK");
    private double BLOCK_ONE_FORWARD_TO_FOUNDATION = propertiesLoader.getDoubleProperty("BLOCK_ONE_FORWARD_TO_FOUNDATION");
    private double BLOCK_TWO_FORWARD_TO_FOUNDATION = propertiesLoader.getDoubleProperty("BLOCK_TWO_FORWARD_TO_FOUNDATION");
    private double BLOCK_THREE_FORWARD_TO_FOUNDATION = propertiesLoader.getDoubleProperty("BLOCK_THREE_FORWARD_TO_FOUNDATION");
    private double BLOCK_FOUR_BACKWARD_TO_BLOCK = propertiesLoader.getDoubleProperty("BLOCK_FOUR_BACKWARD_TO_BLOCK");
    private double BLOCK_FIVE_BACKWARD_TO_BLOCK = propertiesLoader.getDoubleProperty("BLOCK_FIVE_BACKWARD_TO_BLOCK");
    private double BLOCK_SIX_BACKWARD_TO_BLOCK = propertiesLoader.getDoubleProperty("BLOCK_SIX_BACKWARD_TO_BLOCK");
    private double BLOCK_FOUR_FORWARD_TO_FOUNDATION = propertiesLoader.getDoubleProperty("BLOCK_FOUR_FORWARD_TO_FOUNDATION");
    private double BLOCK_FIVE_FORWARD_TO_FOUNDATION = propertiesLoader.getDoubleProperty("BLOCK_FIVE_FORWARD_TO_FOUNDATION");
    private double BLOCK_SIX_FORWARD_TO_FOUNDATION = propertiesLoader.getDoubleProperty("BLOCK_SIX_FORWARD_TO_FOUNDATION");
    private double BLOCK_EXTRA_DISTANCE_HORIZONTAL_FOUNTATION = propertiesLoader.getDoubleProperty("BLOCK_EXTRA_DISTANCE_HORIZONTAL_FOUNTATION");
    private double BLOCK_STRAFE_DIST = propertiesLoader.getDoubleProperty("BLOCK_STRAFE_DIST");
    private double BLOCK_STRAFE_DIFF_DIST = propertiesLoader.getDoubleProperty("BLOCK_STRAFE_DIST_DIFF");
    private double BLOCK_STRAFE_DIST_2 = propertiesLoader.getDoubleProperty("BLOCK_STRAFE_DIST_2");
    private double BLOCK_STRAFE_DIFF_DIST_2 = propertiesLoader.getDoubleProperty("BLOCK_STRAFE_DIST_DIFF_2");
    private double horizontalTurnDegree = 90;
    private double verticalTurnDegree = 90;
    private double fiddleDistance = -3;
    private double blockRotation = -90;
    private volatile boolean flag = false;
    private ElapsedTime runtime = new ElapsedTime();
    private RobotLinearOpMode robot;
    private FOUNDATION_ORIENTATION foundationOrientation;
    private PARK_POSITION parkPosition;
    private SIDE side;
    private ALLIANCE alliance;
    private LinearOpMode opMode;
    private SKYSTONE_POSITION skystone_position;
    private StageSwitchingPipeline pipeline;
    private int checks = 0;

    private double BLOCK_TO_BRIDGE = propertiesLoader.getDoubleProperty("BLOCK_TO_BRIDGE");
    private double BLOCK_EXTRA_DIST_TO_FOUNDATION = propertiesLoader.getDoubleProperty("BLOCK_EXTRA_DIST_TO_FOUNDATION");

    public DWAIAutonomous(
            FOUNDATION_ORIENTATION foundationOrientation,
            PARK_POSITION parkPosition,
            SIDE side,
            ALLIANCE alliance,
            LinearOpMode opMode
    ){
        this.foundationOrientation = foundationOrientation;
        this.parkPosition          = parkPosition;
        this.side                  = side;
        this.alliance              = alliance;
        this.opMode                = opMode;
    }

    public void runOpMode(){
        robot = new RobotLinearOpMode(opMode);
        setupVariables();

        if (side == SIDE.BLOCK) {
            robot.setPattern(BlinkinPattern.BLACK);
            openCVinit();
        }

        robot.setLatchPosition(OPEN_LATCH_SERVO_POSITION);
        opMode.waitForStart();
        runtime.reset();

        if (side == SIDE.FOUNDATION) {

            robot.setPattern(BlinkinPattern.RAINBOW_WITH_GLITTER);
            moveToFoundation();
            latchFoundation();

            if (foundationOrientation == FOUNDATION_ORIENTATION.HORIZONTAL) {
                placeHorizontal();
                parkHorizontal();
            }
            else {
                placeVertical();
                parkVertical();
            }

            robot.stopDriveMotors();
        }
        else if (side == SIDE.BLOCK) {

            while(skystone_position == null || checks < 10){
                getSkyStonePosition();
                checks++;
                opMode.sleep(100);
            }

            robot.setPattern(BlinkinPattern.RAINBOW_WITH_GLITTER);

            Thread deploy = new Thread(() -> {
                startDeploy();
                flag = true;
            });

            deploy.start();

            print("Lowering grabber");
            setGrabberPosition(READY);
            goToBlock();
            robot.stopDriveMotors();

            print("Strafing against block");
            robot.compensatingMoveByInchesFast(BLOCK_STRAFE_DIST, STRAFE, blockRotation);

            print("Grabbing block");
            setGrabberPosition(GRABBING);
            opMode.sleep(500);
            print("Stowing block");
            setGrabberPosition(STOWED);
            robot.compensatingMoveByInchesFast(-(BLOCK_STRAFE_DIST + BLOCK_STRAFE_DIFF_DIST), STRAFE, blockRotation);

            waitForFlag();
            forwardToFoundationFirstTime();

            //TODO slow this down so that it does thing well
            robot.stopDriveMotors();
            print("Dropping block off");
            setGrabberPosition(GRABBING);
            opMode.sleep(500);
            setGrabberPosition(READY);
            opMode.sleep(500);
            print("Raising to starting position");
            Thread t1 = new Thread(() -> {

                try{
                    sleep(500);
                } catch(Exception e){

                }

                setGrabberPosition(STOWED);
            });

            t1.start();

            backwardToBlocks();
            robot.stopDriveMotors();
            //robot.turnToDegreeFast(blockRotation);

            print("Lowering grabber");
            setGrabberPosition(READY);
            opMode.sleep(500);

            print("Strafing against block");
            robot.moveByInchesFast(BLOCK_STRAFE_DIST_2, STRAFE);

            robot.stopDriveMotors();
            print("Grabbing block");
            setGrabberPosition(GRABBING);
            opMode.sleep(500);
            setGrabberPosition(STOWED);

            robot.moveByInchesFast(-(BLOCK_STRAFE_DIST_2 + BLOCK_STRAFE_DIFF_DIST_2), STRAFE);

            forwardToFoundationSecondTime();

            robot.stopDriveMotors();
            print("Dropping block off");
            setGrabberPosition(GRABBING);
            opMode.sleep(500);
            setGrabberPosition(READY);
            Thread t2 = new Thread(() -> {

                try{
                    sleep(500);
                } catch(Exception e){

                }

                setGrabberPosition(STOWED);
            });

            t2.start();
            print("Parking");
            robot.moveByInchesFast(BLOCK_TO_BRIDGE, FORWARD);
            print("Raising to starting position");
            setGrabberPosition(DEFAULT);
            opMode.sleep(500);
        }
        else if (side == SIDE.QUICK_PARK || side == SIDE.SLOW_PARK) {
            if(side == SIDE.SLOW_PARK){
                opMode.sleep(25000);
            }
            startDeploy();
            robot.moveByInchesFast(12, FORWARD);
            robot.stopDriveMotors();
        }

        AutoTransitioner.transitionOnStop(opMode, "Skystone Main Teleop", alliance);
    }

    private void setGrabberPosition(RobotLinearOpMode.GRABBER_POSITION pos){

        switch (alliance) {
            case RED:
                robot.setLeftGrabberPosition(pos);
                break;
            case BLUE:
                robot.setRightGrabberPosition(pos);
                break;
        }

    }

    private void waitForFlag(){
        print("Waiting for flag");
        while(opMode.opModeIsActive() && !flag){
        }
        print("Done waiting");
        flag = false;
    }

    private void print(String printString){
        opMode.telemetry.addLine(printString);
        opMode.telemetry.update();

        if (PAUSE_STEPS) {
            robot.stopAllMotors();
            //Ensures button is pressed and released before continuing
            while(opMode.opModeIsActive() && !opMode.gamepad1.a){
            }
            while(opMode.opModeIsActive() && opMode.gamepad1.a){
            }
        }

    }

    private void setupVariables(){
        //Necessary since all variables tuned to blue side

        if (alliance == ALLIANCE.RED && side == SIDE.FOUNDATION) {
            DRIVETRAIN_DISTANCE_RIGHT_TO_GET_FOUNDATION *= -1;
            horizontalTurnDegree *= -1;
            fiddleDistance *= -1;
            HWALL_PARK_STRAFE_DISTANCE *= -1;
            HBRIDGE_PARK_STRAFE_DISTANCE *= -1;

            verticalTurnDegree *= -1;
            DRIVETRAIN_DISTANCE_LEFT_TO_CLEAR_FOUNDATION *= -1;
            VWALL_PARK_STRAFE_DISTANCE *= -1;
            VBRIDGE_PARK_STRAFE_DISTANCE *= -1;
        }
        else if ((alliance == ALLIANCE.RED && side == SIDE.BLOCK)) {
            blockRotation *= -1;
            BLOCK_STRAFE_DIST *= -1;
            BLOCK_STRAFE_DIFF_DIST *= -1;
            BLOCK_STRAFE_DIST_2 *= -1;
            BLOCK_STRAFE_DIFF_DIST_2 *= -1;
        }

    }

    private void startDeploy(){
        robot.setLiftPower(-0.5);
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.setLiftPower(0);

        robot.setArmPower(.2);
        try {
            sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.setArmPower(0);

        robot.setLiftPower(0.2);
        robot.setArmPower(-.2);

        try {
            sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        robot.setLiftPower(0);
        robot.setArmPower(0);

        robot.setLiftZeroPowerProperty(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.setArmZeroPowerProperty(DcMotor.ZeroPowerBehavior.FLOAT);

        try {
            sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        robot.setLiftRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setArmRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.setLiftRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setArmRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void moveToFoundation(){
        print("Moving towards foundation");
        robot.moveByInchesFast(DRIVETRAIN_DISTANCE_BACKWARD_TO_GET_OFF_WALL, FORWARD);
        robot.moveByInchesFast(DRIVETRAIN_DISTANCE_RIGHT_TO_GET_FOUNDATION, STRAFE);
        robot.moveByInchesFast(DRIVETRAIN_DISTANCE_BACKWARD_TO_GET_FOUNDATION, FORWARD);
    }

    private void latchFoundation(){
        print("Latching foundation");
        robot.setLatchPosition(OPEN_LATCH_SERVO_POSITION);

        while(!robot.getFoundationSensorPressed() && opMode.opModeIsActive()){
            robot.mecanumPowerDrive(0, -.3, 0);
        }

        print("Latched foundation");
        robot.setLatchPosition(CLOSE_LATCH_SERVO_POSITION);
        robot.moveByInchesFast(-2, FORWARD);
    }

    private void placeHorizontal(){
        print("Driving forwards");
        robot.moveByInchesFast(DRIVETRAIN_DISTANCE_FORWARD_TO_TURN, FORWARD);

        print("Turning to be forward");
        robot.turnToDegreeFast(horizontalTurnDegree);
        robot.stopDriveMotors();
        robot.setLatchPosition(OPEN_LATCH_SERVO_POSITION);
    }

    private void parkHorizontal(){
        print("Fiddling with latch");
        robot.moveByInchesFast(fiddleDistance, STRAFE);

        if (parkPosition == PARK_POSITION.WALL) {
            print("Strafing against wall");
            robot.moveByInchesFast(HWALL_PARK_STRAFE_DISTANCE, STRAFE);
        }
        else {
            print("Strafing towards bridge");
            robot.moveByInchesFast(HBRIDGE_PARK_STRAFE_DISTANCE, STRAFE);
        }

        robot.stopDriveMotors();
        print("Deploying intake");
        startDeploy();

        print("Moving under bridge");
        robot.moveByInchesFast(36, FORWARD);
        //rough movement, likely replace with color sensor line code
    }

    private void placeVertical(){
        print("Driving forwards");
        robot.moveByInchesFast(DRIVETRAIN_DISTANCE_FORWARD_TO_DEPOT, FORWARD);

        print("Opening latch");
        robot.setLatchPosition(OPEN_LATCH_SERVO_POSITION);

        print("Strafing away from the platform");
        robot.moveByInchesFast(DRIVETRAIN_DISTANCE_LEFT_TO_CLEAR_FOUNDATION, STRAFE);

        print("Moving up parallel with platform");
        robot.moveByInchesFast(DRIVETRAIN_DISTANCE_BACKWARD_TO_MIDDLE_OF_FOUNDATION, FORWARD);

        print("Turning to be forward");
        robot.turnToDegreeFast(verticalTurnDegree);

        print("Slamming foundation against the wall really really hard");
        robot.moveByInchesFast(DRIVETRAIN_DISTANCE_BACKWARD_TO_SLAM_FOUNDATION_REALLY_REALLY_HARD, FORWARD);
    }

    private void parkVertical(){

        if (parkPosition == PARK_POSITION.WALL) {
            print("Strafing against wall");
            robot.moveByInchesFast(VWALL_PARK_STRAFE_DISTANCE, STRAFE);
        }
        else {
            print("Strafing against bridge");
            robot.moveByInchesFast(VBRIDGE_PARK_STRAFE_DISTANCE, STRAFE);
        }

        robot.stopDriveMotors();
        print("Deploying intake");
        startDeploy();

        print("Moving under bridge");
        robot.moveByInchesFast(22, FORWARD);
    }

    private void getSkyStonePosition(){
        int valLeft;
        int valMid;
        int valRight;

        int[] vals = pipeline.getPositions();
        valLeft = vals[0];
        valMid = vals[1];
        valRight = vals[2];

        if (alliance == ALLIANCE.BLUE) {
            if (valLeft == 0) skystone_position = SKYSTONE_POSITION.THREE_AND_SIX;
            if (valMid == 0) skystone_position = SKYSTONE_POSITION.TWO_AND_FIVE;
            if (valRight == 0) skystone_position = SKYSTONE_POSITION.ONE_AND_FOUR;
        }
        else {
            if (valLeft == 0) skystone_position = SKYSTONE_POSITION.ONE_AND_FOUR;
            if (valMid == 0) skystone_position = SKYSTONE_POSITION.TWO_AND_FIVE;
            if (valRight == 0) skystone_position = SKYSTONE_POSITION.THREE_AND_SIX;
        }

        if (skystone_position == null) {
            //throw new RuntimeException("No skystone detected!");
            opMode.telemetry.addLine("No Skystone detected!");
            opMode.telemetry.update();
        }
        else {
            opMode.telemetry.addLine(skystone_position.name());
            opMode.telemetry.update();
        }

    }

    private void goToBlock(){
        double dist = 0;

        //strafing to in front of the block
        switch (skystone_position) {
            case ONE_AND_FOUR:
                dist = BLOCK_ONE_FORWARD_TO_BLOCK;
                break;
            case TWO_AND_FIVE:
                dist = BLOCK_TWO_FORWARD_TO_BLOCK;
                break;
            case THREE_AND_SIX:
                dist = BLOCK_THREE_FORWARD_TO_BLOCK;
                break;
        }

        print("Going to block");
        robot.moveByInchesFast(BLOCK_FORWARD_OFF_WALL_TO_BLOCK, FORWARD);
        robot.turnToDegreeFast(blockRotation);

        if (dist != 0) {
            robot.moveByInchesFast(dist, FORWARD);
        }

    }

    private void forwardToFoundationFirstTime(){
        //runIntake(-1);

        double forwardDistance = 0;

        switch (skystone_position) {
            case ONE_AND_FOUR:

                forwardDistance = BLOCK_ONE_FORWARD_TO_FOUNDATION;
                break;
            case TWO_AND_FIVE:

                forwardDistance = BLOCK_TWO_FORWARD_TO_FOUNDATION;
                break;
            case THREE_AND_SIX:

                forwardDistance = BLOCK_THREE_FORWARD_TO_FOUNDATION;
                break;
        }

        //if (foundationOrientation == FOUNDATION_ORIENTATION.HORIZONTAL) {
        forwardDistance += BLOCK_EXTRA_DISTANCE_HORIZONTAL_FOUNTATION;
        //}

        print("Moving backwards to foundation");

        if(foundationOrientation == FOUNDATION_ORIENTATION.VERTICAL){
            forwardDistance += BLOCK_EXTRA_DIST_TO_FOUNDATION;
        }

        //robot.moveByInchesFast(forwardDistance, FORWARD);
        robot.compensatingMoveByInchesFast(forwardDistance, FORWARD, blockRotation);
    }

    private void backwardToBlocks(){
        double backwardDistance = 0;

        switch (skystone_position) {
            case ONE_AND_FOUR:

                backwardDistance = BLOCK_FOUR_BACKWARD_TO_BLOCK;
                break;
            case TWO_AND_FIVE:

                backwardDistance = BLOCK_FIVE_BACKWARD_TO_BLOCK;
                break;
            case THREE_AND_SIX:

                backwardDistance = BLOCK_SIX_BACKWARD_TO_BLOCK;
                break;
        }

        //if (foundationOrientation == FOUNDATION_ORIENTATION.HORIZONTAL) {
        backwardDistance -= BLOCK_EXTRA_DISTANCE_HORIZONTAL_FOUNTATION;
        //}

        if(foundationOrientation == FOUNDATION_ORIENTATION.VERTICAL){
            backwardDistance -= BLOCK_EXTRA_DIST_TO_FOUNDATION;
        }

        print("Moving forwards to blocks");
        //robot.moveByInchesFast(backwardDistance, FORWARD);
        robot.compensatingMoveByInchesFast(backwardDistance, FORWARD, blockRotation);
    }

    private void forwardToFoundationSecondTime(){
        double forwardDistance = 0;
        switch (skystone_position) {
            case ONE_AND_FOUR:
                forwardDistance = BLOCK_FOUR_FORWARD_TO_FOUNDATION;
                break;
            case TWO_AND_FIVE:
                forwardDistance = BLOCK_FIVE_FORWARD_TO_FOUNDATION;
                break;
            case THREE_AND_SIX:
                forwardDistance = BLOCK_SIX_FORWARD_TO_FOUNDATION;
                break;
        }

        //if (foundationOrientation == FOUNDATION_ORIENTATION.HORIZONTAL) {
            forwardDistance += BLOCK_EXTRA_DISTANCE_HORIZONTAL_FOUNTATION;
        //}

        if(foundationOrientation == FOUNDATION_ORIENTATION.VERTICAL){
            forwardDistance += BLOCK_EXTRA_DIST_TO_FOUNDATION;
        }

        print("Moving backwards to foundation");
        //robot.moveByInchesFast(forwardDistance, FORWARD);
        robot.compensatingMoveByInchesFast(forwardDistance, FORWARD, blockRotation);
    }

    private void openCVinit(){
        float offsetX         = propertiesLoader.getFloatProperty("OFFSET_X");//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
        float allianceOffsetX = propertiesLoader.getFloatProperty("ALLIANCE_OFFSET_X");//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
        float offsetY         = propertiesLoader.getFloatProperty("OFFSET_Y");//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive
        float distScale       = propertiesLoader.getFloatProperty("DIST_SCALE");

        if (alliance == ALLIANCE.BLUE) {
            offsetX -= allianceOffsetX;
        }

        //offset array: 0 = x, 1 = y, 2 = distScale
        float[] offsetArray = {offsetX, offsetY, distScale};
        pipeline = new StageSwitchingPipeline(offsetArray);

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        OpenCvCamera phoneCam            = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(pipeline);//different stages
        int rows = 640;
        int cols = 480;
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.
    }

    public enum FOUNDATION_ORIENTATION {
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
        FOUNDATION,
        QUICK_PARK,
        SLOW_PARK
    }

    enum SKYSTONE_POSITION {
        ONE_AND_FOUR,
        TWO_AND_FIVE,
        THREE_AND_SIX,
    }

}*/

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.IO.PropertiesLoader;
import org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode;
import org.firstinspires.ftc.teamcode.Robot.StageSwitchingPipeline;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

/**
 * Perhaps try implementing optimized rev drive
 */
public class DWAIAutonomous {

    private PropertiesLoader propertiesLoader = new PropertiesLoader("Autonomous");
    private double BLOCK_OFFSET_Y_POSITION = propertiesLoader.getDoubleProperty("BLOCK_OFFSET_Y_POSITION");
    private double BLOCK_Y_POSITION = propertiesLoader.getDoubleProperty("BLOCK_Y_POSITION");
    private double BRIDGE_Y_POSITION = propertiesLoader.getDoubleProperty("BRIDGE_Y_POSITION");
    private double FOUNDATION_Y_POSITION = propertiesLoader.getDoubleProperty("FOUNDATION_Y_POSITION");
    private double DEPOT_Y_POSITION = propertiesLoader.getDoubleProperty("DEPOT_Y_POSITION");

    private double OPEN_LATCH_SERVO_POSITION = propertiesLoader.getDoubleProperty("OPEN_LATCH_SERVO_POSITION");
    private double CLOSE_LATCH_SERVO_POSITION = propertiesLoader.getDoubleProperty("CLOSE_LATCH_SERVO_POSITION");
    private long GRAB_DELAY = propertiesLoader.getLongProperty("GRAB_DELAY");
    private boolean PAUSE_STEPS = propertiesLoader.getBooleanProperty("PAUSE_STEPS");
    private boolean DEPLOY_LIFTER = propertiesLoader.getBooleanProperty("DEPLOY_LIFTER");

    private int checks = 0;
    private StageSwitchingPipeline pipeline;
    private OpenCvCamera phoneCam;

    private SampleMecanumDriveREV drive;
    private RobotLinearOpMode robot;
    private FOUNDATION_ORIENTATION foundationOrientation;
    private PARK_POSITION parkPosition;
    private SIDE side;
    private ALLIANCE alliance;
    private SKYSTONE_POSITION skystone_position;
    private LinearOpMode opMode;
    private Telemetry telemetry;

    private double startAngle = 0;
    private int blocksPlaced = 0;

    private volatile boolean flag = false;

    public DWAIAutonomous(
            FOUNDATION_ORIENTATION foundationOrientation,
            PARK_POSITION parkPosition,
            SIDE side,
            ALLIANCE alliance,
            LinearOpMode opMode
    ){
        this.foundationOrientation = foundationOrientation;
        this.parkPosition          = parkPosition;
        this.side                  = side;
        this.alliance              = alliance;
        this.opMode                = opMode;
        this.telemetry             = opMode.telemetry;
    }

    private void print(String printString){
        opMode.telemetry.addLine(printString);
        opMode.telemetry.update();

        if (PAUSE_STEPS) {
            robot.stopAllMotors();
            //Ensures button is pressed and released before continuing
            while(opMode.opModeIsActive() && !opMode.gamepad1.a);
            while(opMode.opModeIsActive() && opMode.gamepad1.a);
        }

    }

    private void setupVariables(){

        if(alliance == ALLIANCE.RED){
            BLOCK_Y_POSITION *= -1;
            BLOCK_OFFSET_Y_POSITION *= -1;
            BRIDGE_Y_POSITION *= -1;
            FOUNDATION_Y_POSITION *= -1;
            DEPOT_Y_POSITION *= -1;
        }

    }

    public void runOpMode(){
        robot = new RobotLinearOpMode(opMode);
        drive = new SampleMecanumDriveREV(opMode.hardwareMap);
        setupVariables();

        //Additional setup
        robot.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        openCVinit();

        telemetry.addLine("Initialized!");
        telemetry.update();

        opMode.waitForStart();

        //Some roadrunner stuff
        robot.setLeftGrabberPosition(RobotLinearOpMode.GRABBER_POSITION.DEFAULT);
        robot.setRightGrabberPosition(RobotLinearOpMode.GRABBER_POSITION.DEFAULT);
        robot.setLatchPosition(OPEN_LATCH_SERVO_POSITION);

        if(alliance == ALLIANCE.RED && side == SIDE.BLOCK){
            drive.setPoseEstimate(new Pose2d(-34, -63, Math.toRadians(270)));
            startAngle = 270;
        } else if(alliance == ALLIANCE.RED && side == SIDE.FOUNDATION){
            drive.setPoseEstimate(new Pose2d(15, -63, Math.toRadians(270)));
            startAngle = 270;
        } else if(alliance == ALLIANCE.BLUE && side == SIDE.BLOCK){
            drive.setPoseEstimate(new Pose2d(-34, 63, Math.toRadians(90)));
            startAngle = 90;
        } else if(alliance == ALLIANCE.BLUE && side == SIDE.FOUNDATION){
            drive.setPoseEstimate(new Pose2d(15, 63, Math.toRadians(90)));
            startAngle = 90;
        }

        if(side == SIDE.BLOCK) {
            executeBlockAuto();
        } else if(side == SIDE.FOUNDATION){
            executeFoundationAuto();
        } else if(side == SIDE.QUICK_PARK){
            //Deploy lifter
            Thread deploy = new Thread(() -> {
                startDeploy();
                flag = true;
            });

            deploy.start();
            while(!flag && opMode.opModeIsActive());

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(18)
                            .build()
            );

        } else if(side == SIDE.SLOW_PARK){
            //Deploy lifter
            Thread deploy = new Thread(() -> {
                startDeploy();
                flag = true;
            });

            deploy.start();
            opMode.sleep(25000);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(18)
                            .build()
            );

        }

        //AutoTransitioner.transitionOnStop(opMode, "Skystone Main Teleop", alliance);
    }

    private void executeBlockAuto(){
        /*//Detect block
        while (skystone_position == null || checks < 10) {
            getSkyStonePosition();
            checks++;
            opMode.sleep(100);
        }*/
        //phoneCam.closeCameraDevice();
        skystone_position = SKYSTONE_POSITION.ONE_AND_FOUR; //testing purposes

        robot.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);

        //Move off wall
        /*drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(12)
                        .build()
        );*/

        //Deploy lifter
        Thread deploy = new Thread(() -> {
            startDeploy();
            flag = true;
        });

        deploy.start();

        setGrabberPos(RobotLinearOpMode.GRABBER_POSITION.READY);
        print("Moving to block");

        //PERHAPS TRY USING STRAFETO
        //replace with BLOCK_OFFSET_Y_POSITION to enable strafing
        switch(skystone_position) {
            case ONE_AND_FOUR:
                //Block 1 position
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(-48, BLOCK_OFFSET_Y_POSITION, 0))
                                .reverse()
                                .splineTo(new Pose2d(-20, BLOCK_OFFSET_Y_POSITION, 0))
                                .strafeRight((BLOCK_OFFSET_Y_POSITION - BLOCK_Y_POSITION))
                                .build()
                );
                break;
            case TWO_AND_FIVE:
                //Block 2 position
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(-48, BLOCK_OFFSET_Y_POSITION, 0))
                                .reverse()
                                .splineTo(new Pose2d(-28, BLOCK_OFFSET_Y_POSITION, 0))
                                .strafeRight((BLOCK_OFFSET_Y_POSITION - BLOCK_Y_POSITION))
                                .build()
                );
                break;
            case THREE_AND_SIX:
                //Block 3 position
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(-48, BLOCK_OFFSET_Y_POSITION, 0))
                                .reverse()
                                .splineTo(new Pose2d(-36, BLOCK_OFFSET_Y_POSITION, 0))
                                .strafeRight((BLOCK_OFFSET_Y_POSITION - BLOCK_Y_POSITION))
                                .build()
                );
                break;
        }

        print("Grabbing block");
        setGrabberPos(RobotLinearOpMode.GRABBER_POSITION.GRABBING);
        opMode.sleep(GRAB_DELAY);
        setGrabberPos(RobotLinearOpMode.GRABBER_POSITION.STOWED);
        opMode.sleep(GRAB_DELAY);

        while(!flag && opMode.opModeIsActive());
        placeBlock();

        switch(skystone_position){
            case ONE_AND_FOUR:
                grabBlock(4);
                break;
            case TWO_AND_FIVE:
                grabBlock(5);
                break;
            case THREE_AND_SIX:
                grabBlock(6);
                break;
        }

        placeBlock();
        dragFoundation();

        //grab more block?

        telemetry.addLine("Done!");
        telemetry.update();

        //X diffs
        //POS 1: -20, 33.3
        //POS 2: -28, 33.3
        //POS 3: -36, 33.3

        //BRIDGE: 0, 40 < go here between blocks
        //FOUNDATION: 45, 35

        //X diffs
        //POS 4: -44, 33.3
        //POS 5: -52, 33.3
        //POS 6: -60, 33.3

        //turn to 90 at end
        //latch
        //move to 60, 63
        //park at 0, 38
    }

    private void dragFoundation(){
        print("Latching foundation");

        drive.turnSync(Math.toRadians(180 - startAngle));

        while(!robot.getFoundationSensorPressed()){
            robot.mecanumPowerDrive(0, -0.2, 0);
            drive.updatePoseEstimate();
        }

        robot.mecanumPowerDrive(0, 0, 0);
        robot.setLatchPosition(CLOSE_LATCH_SERVO_POSITION);

        opMode.sleep(GRAB_DELAY);

        //WILLIAM
        //make it move to where it needs to be, turn, and then move backwards instead of singular spline
        //the angles are in radians, limited to between 0, 2pi, positive only
        //connect using 192.168.49.1:8080/dash, switch graph to field from Default to see positioning

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(45, DEPOT_Y_POSITION, Math.toRadians(180)))
                        .build()
        );

        robot.setLatchPosition(OPEN_LATCH_SERVO_POSITION);
    }

    private void placeBlock(){
        print("Going under bridge");

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0,  BRIDGE_Y_POSITION, 0))
                        .splineTo(new Pose2d(48 + blocksPlaced * 9, FOUNDATION_Y_POSITION, 0))
                        .build()
        );

        print("Placing block");
        setGrabberPos(RobotLinearOpMode.GRABBER_POSITION.GRABBING);
        opMode.sleep(GRAB_DELAY);
        setGrabberPos(RobotLinearOpMode.GRABBER_POSITION.READY);
        opMode.sleep(GRAB_DELAY);
        setGrabberPos(RobotLinearOpMode.GRABBER_POSITION.DEFAULT);
        blocksPlaced++;
    }

    private void grabBlock(int index){
        print("Going back for another one");
        setGrabberPos(RobotLinearOpMode.GRABBER_POSITION.READY);

        //to enable strafe, replace BLOCK_Y_POSITION with BLOCK_OFFSET_Y_POSITION in splineto (-12)
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(0, BRIDGE_Y_POSITION, 0))
                        .splineTo(new Pose2d(-12 - index * 8, BLOCK_OFFSET_Y_POSITION, 0))
                        .strafeRight((BLOCK_OFFSET_Y_POSITION - BLOCK_Y_POSITION))
                        .build()
        );

        print("Grabbing block");
        setGrabberPos(RobotLinearOpMode.GRABBER_POSITION.GRABBING);
        opMode.sleep(GRAB_DELAY);
        setGrabberPos(RobotLinearOpMode.GRABBER_POSITION.STOWED);
        opMode.sleep(GRAB_DELAY);
    }

    private void executeFoundationAuto(){

        //make heading a variable
        // make y positions a variable

        //strafe to 40, 63
        //back up to ~40, 35
        //latch
        //turn to 180
        //strafe to 40, 63
        //unlatch
        //park at 0, 63
    }

    private void setGrabberPos(RobotLinearOpMode.GRABBER_POSITION pos){

        switch(alliance){
            case BLUE:
                robot.setRightGrabberPosition(pos);
                break;
            case RED:
                robot.setLeftGrabberPosition(pos);
                break;
        }

    }

    private void startDeploy(){

        if(!DEPLOY_LIFTER){
            return;
        }

        robot.setLiftPower(-0.5);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.setLiftPower(0);

        robot.setArmPower(.2);
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        robot.setArmPower(0);

        robot.setLiftPower(0.2);
        robot.setArmPower(-.2);

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        robot.setLiftPower(0);
        robot.setArmPower(0);

        robot.setLiftZeroPowerProperty(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.setArmZeroPowerProperty(DcMotor.ZeroPowerBehavior.FLOAT);

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        robot.setLiftRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setArmRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.setLiftRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setArmRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void getSkyStonePosition(){
        int valLeft;
        int valMid;
        int valRight;

        int[] vals = pipeline.getPositions();
        valLeft = vals[0];
        valMid = vals[1];
        valRight = vals[2];

        if (alliance == ALLIANCE.BLUE) {
            if (valLeft == 0) skystone_position = SKYSTONE_POSITION.THREE_AND_SIX;
            if (valMid == 0) skystone_position = SKYSTONE_POSITION.TWO_AND_FIVE;
            if (valRight == 0) skystone_position = SKYSTONE_POSITION.ONE_AND_FOUR;
        } else {
            if (valLeft == 0) skystone_position = SKYSTONE_POSITION.ONE_AND_FOUR;
            if (valMid == 0) skystone_position = SKYSTONE_POSITION.TWO_AND_FIVE;
            if (valRight == 0) skystone_position = SKYSTONE_POSITION.THREE_AND_SIX;
        }

        if (skystone_position == null) {
            //throw new RuntimeException("No skystone detected!");
            telemetry.addLine("No Skystone detected!");
            telemetry.update();
        } else {
            telemetry.addLine(skystone_position.name());
            telemetry.update();
        }

    }

    private void openCVinit(){
        float offsetX         = propertiesLoader.getFloatProperty("OFFSET_X");//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
        float allianceOffsetX = propertiesLoader.getFloatProperty("ALLIANCE_OFFSET_X");//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
        float offsetY         = propertiesLoader.getFloatProperty("OFFSET_Y");//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive
        float distScale       = propertiesLoader.getFloatProperty("DIST_SCALE");//changing this scales the distance between everything by a factor

        if (alliance == ALLIANCE.BLUE) {
            offsetX -= allianceOffsetX;
        }

        //offset array: 0 = x, 1 = y, 2 = distScale
        float[] offsetArray = {offsetX, offsetY, distScale};
        pipeline = new StageSwitchingPipeline(offsetArray);

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(pipeline);//different stages
        int rows = 640;
        int cols = 480;
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//stream on phone
    }

    enum SKYSTONE_POSITION {
        ONE_AND_FOUR,
        TWO_AND_FIVE,
        THREE_AND_SIX,
    }

    public enum FOUNDATION_ORIENTATION {
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
        FOUNDATION,
        QUICK_PARK,
        SLOW_PARK
    }

}
