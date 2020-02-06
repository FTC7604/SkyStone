package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.IO.PropertiesLoader;
import org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.GRABBER_POSITION.GRABBING;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.GRABBER_POSITION.READY;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.GRABBER_POSITION.STOWED;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.STRAFE;


/**
 * IMPORTANT NOTES
 * FOR BLOCK AUTO: VERTICAL = ONE-BLOCK, HORIZONTAL = TWO-BLOCK AT THIS POINT IN TIME
 * LIKELY WILL BE CHANGED BACK FOR FUTURE TOURNAMENTS
 * MUST CHANGE TO + FIGURE OUT ODOMETRY BECAUSE FIELD TILE FRICTION CHANGES
 * WOULD BE VERY NICE TO HAVE A SERVO CLAW ON THE SIDE (QUICKER SAMPLING)
 * REMEMBER TO MAKE A BRIDGE PARK AUTO (park is only against the wall right now)
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

public class DWAIAutonomous {

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;
    private static float[] midPos = new float[2];
    private static float[] leftPos = new float[2];
    private static float[] rightPos = new float[2];
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
    private double BLOCK_ONE_STRAFE_TO_BLOCK = propertiesLoader.getDoubleProperty("BLOCK_ONE_FORWARD_TO_BLOCK");
    private double BLOCK_TWO_STRAFE_TO_BLOCK = propertiesLoader.getDoubleProperty("BLOCK_TWO_FORWARD_TO_BLOCK");
    private double BLOCK_THREE_STRAFE_TO_BLOCK = propertiesLoader.getDoubleProperty("BLOCK_THREE_FORWARD_TO_BLOCK");
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

    private int checks = 0;

    private double BLOCK_TO_BRIDGE = propertiesLoader.getDoubleProperty("BLOCK_TO_BRIDGE");;

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
            robot.moveByInchesFast(BLOCK_STRAFE_DIST, STRAFE);

            print("Grabbing block");
            setGrabberPosition(GRABBING);
            opMode.sleep(500);
            print("Stowing block");
            setGrabberPosition(STOWED);
            robot.moveByInchesFast(-BLOCK_STRAFE_DIST, STRAFE);

            waitForFlag();
            forwardToFoundationFirstTime();

            //TODO slow this down so that it does thing well
            robot.stopDriveMotors();
            print("Dropping block off");
            setGrabberPosition(GRABBING);
            opMode.sleep(500);
            setGrabberPosition(READY);
//            opMode.sleep(1000);
//            print("Raising to starting position");
//            setGrabberPosition(DEFAULT);

            backwardToBlocks();

            robot.stopDriveMotors();

            print("Strafing against block");
            robot.moveByInchesFast(BLOCK_STRAFE_DIST_2, STRAFE);

            robot.stopDriveMotors();
            print("Grabbing block");
            setGrabberPosition(GRABBING);
            opMode.sleep(500);
            print("Stowing block");
            setGrabberPosition(STOWED);
            robot.moveByInchesFast(-(BLOCK_STRAFE_DIST_2 + BLOCK_STRAFE_DIFF_DIST_2), STRAFE);

            forwardToFoundationSecondTime();

            robot.stopDriveMotors();
            print("Dropping block off");
            setGrabberPosition(GRABBING);
            opMode.sleep(500);
            setGrabberPosition(READY);
//            opMode.sleep(1000);
//            print("Raising to starting position");
//            setGrabberPosition(DEFAULT);

            robot.moveByInchesFast(BLOCK_TO_BRIDGE, FORWARD);

            /*strafeToBlock();
            grabBlockFirstTime();
            forwardToFoundationFirstTime();

            if(foundationOrientation == FOUNDATION_ORIENTATION.HORIZONTAL) {
                dropOffBlock(true);
                backwardToBlocks();
                grabBlockSecondTime();
                forwardToFoundationSecondTime();
                dropOffBlock(false);
                //blockPark();
                robot.stopDriveMotors();
            } else if(foundationOrientation == FOUNDATION_ORIENTATION.VERTICAL){
                dropOffBlock(false);
                robot.stopDriveMotors();
            }*/

        }
        else if (side == SIDE.JUST_PARK) {
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
            case BLUE:
                robot.setRightGrabberPosition(pos);
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
            BLOCK_ONE_STRAFE_TO_BLOCK *= -1;
            BLOCK_TWO_STRAFE_TO_BLOCK *= -1;
            BLOCK_THREE_STRAFE_TO_BLOCK *= -1;
            BLOCK_TO_BRIDGE *= -1;
            blockRotation *= -1;

            BLOCK_STRAFE_DIST *= -1;
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
                dist = BLOCK_ONE_STRAFE_TO_BLOCK;
                break;
            case TWO_AND_FIVE:
                dist = BLOCK_TWO_STRAFE_TO_BLOCK;
                break;
            case THREE_AND_SIX:
                dist = BLOCK_THREE_STRAFE_TO_BLOCK;
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

        if (foundationOrientation == FOUNDATION_ORIENTATION.HORIZONTAL) {
            forwardDistance += BLOCK_EXTRA_DISTANCE_HORIZONTAL_FOUNTATION;
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

        midPos[0]   = (4f + offsetX) / 8f;
        midPos[1]   = (4f + offsetY) / 8f;
        leftPos[0]  = (4f + offsetX - distScale * 2f) / 8f;
        leftPos[1]  = (4f + offsetY) / 8f;
        rightPos[0] = (4f + offsetX + distScale * 2f) / 8f;
        rightPos[1] = (4f + offsetY) / 8f;

        int          cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        OpenCvCamera phoneCam            = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
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
        JUST_PARK
    }

    enum SKYSTONE_POSITION {
        ONE_AND_FOUR,
        TWO_AND_FIVE,
        THREE_AND_SIX,
    }

    //detection pipeline (ignore)
    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();
        private StageSwitchingPipeline.Stage stageToRenderToViewport;
        private StageSwitchingPipeline.Stage[] stages = StageSwitchingPipeline.Stage.values();

        {
            stageToRenderToViewport = StageSwitchingPipeline.Stage.detection;
        }

        @Override
        public void onViewportTapped(){
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            stageToRenderToViewport = StageSwitchingPipeline.Stage.detection;
            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input){
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours

            //get values from frame
            double[] pixMid = thresholdMat.get((int) (input.rows() * midPos[1]), (int) (input.cols() * midPos[0]));//gets value at circle
            valMid = (int) pixMid[0];
            //valMid = radialAverage(midPos, input);

            double[] pixLeft = thresholdMat.get((int) (input.rows() * leftPos[1]), (int) (input.cols() * leftPos[0]));//gets value at circle
            valLeft = (int) pixLeft[0];
            //valMid = radialAverage(leftPos, input);

            double[] pixRight = thresholdMat.get((int) (input.rows() * rightPos[1]), (int) (input.cols() * rightPos[0]));//gets value at circle
            valRight = (int) pixRight[0];
            //valRight = radialAverage(rightPos, input);

            //create three points
            Point pointMid   = new Point((int) (input.cols() * midPos[0]), (int) (input.rows() * midPos[1]));
            Point pointLeft  = new Point((int) (input.cols() * leftPos[0]), (int) (input.rows() * leftPos[1]));
            Point pointRight = new Point((int) (input.cols() * rightPos[0]), (int) (input.rows() * rightPos[1]));

            //draw circles on those points
            int radius = 5;
            Imgproc.circle(all, pointMid, radius, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointLeft, radius, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointRight, radius, new Scalar(255, 0, 0), 1);//draws circle

            //draw 3 rectangles
            float rectHeight = .6f / 8f;
            float rectWidth  = 1.5f / 8f;
            Imgproc.rectangle(//1-3
                              all, new Point(input.cols() * (leftPos[0] - rectWidth / 2), input.rows() * (leftPos[1] - rectHeight / 2)), new Point(input.cols() * (leftPos[0] + rectWidth / 2), input.rows() * (leftPos[1] + rectHeight / 2)), new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                              all, new Point(input.cols() * (midPos[0] - rectWidth / 2), input.rows() * (midPos[1] - rectHeight / 2)), new Point(input.cols() * (midPos[0] + rectWidth / 2), input.rows() * (midPos[1] + rectHeight / 2)), new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                              all, new Point(input.cols() * (rightPos[0] - rectWidth / 2), input.rows() * (rightPos[1] - rectHeight / 2)), new Point(input.cols() * (rightPos[0] + rectWidth / 2), input.rows() * (rightPos[1] + rectHeight / 2)), new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport) {
                case THRESHOLD: {
                    return thresholdMat;
                }

                case detection: {
                    return all;
                }

                default: {
                    return input;
                }
            }
        }

        enum Stage {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

    }
}
