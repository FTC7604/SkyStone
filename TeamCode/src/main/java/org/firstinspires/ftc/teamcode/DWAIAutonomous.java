package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.IO.PropertiesLoader;
import org.firstinspires.ftc.teamcode.Robot.*;
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
import java.util.concurrent.LinkedBlockingQueue;

import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.*;

public class DWAIAutonomous {

    private PropertiesLoader propertiesLoader = new PropertiesLoader("Autonomous");

    private double DRIVETRAIN_DISTANCE_RIGHT_TO_GET_FOUNDATION                        = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_RIGHT_TO_GET_FOUNDATION");
    private double DRIVETRAIN_DISTANCE_BACKWARD_TO_GET_OFF_WALL                       = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_BACKWARD_TO_GET_OFF_WALL");
    private double DRIVETRAIN_DISTANCE_BACKWARD_TO_GET_FOUNDATION                     = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_BACKWARD_TO_GET_FOUNDATION");
    private double DRIVETRAIN_DISTANCE_FORWARD_TO_DEPOT                               = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_FORWARD_TO_DEPOT");
    private double DRIVETRAIN_DISTANCE_LEFT_TO_CLEAR_FOUNDATION                       = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_LEFT_TO_CLEAR_FOUNDATION");
    private double DRIVETRAIN_DISTANCE_BACKWARD_TO_MIDDLE_OF_FOUNDATION               = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_BACKWARD_TO_MIDDLE_OF_FOUNDATION");
    private double DRIVETRAIN_DISTANCE_FORWARD_TO_TURN                                = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_FORWARD_TO_TURN");
    private double DRIVETRAIN_DISTANCE_BACKWARD_TO_SLAM_FOUNDATION_REALLY_REALLY_HARD = propertiesLoader.getDoubleProperty("DRIVETRAIN_DISTANCE_BACKWARD_TO_SLAM_FOUNDATION_REALLY_REALLY_HARD");

    private double  HWALL_PARK_STRAFE_DISTANCE   = propertiesLoader.getDoubleProperty("HWALL_PARK_STRAFE_DISTANCE");
    private double  HBRIDGE_PARK_STRAFE_DISTANCE = propertiesLoader.getDoubleProperty("HBRIDGE_PARK_STRAFE_DISTANCE");
    private double  VWALL_PARK_STRAFE_DISTANCE   = propertiesLoader.getDoubleProperty("VWALL_PARK_STRAFE_DISTANCE");
    private double  VBRIDGE_PARK_STRAFE_DISTANCE = propertiesLoader.getDoubleProperty("VBRIDGE_PARK_STRAFE_DISTANCE");
    private boolean PAUSE_STEPS                  = propertiesLoader.getBooleanProperty("PAUSE_STEPS");
    private double  STRAFE_MAX_POWER             = propertiesLoader.getDoubleProperty("STRAFE_MAX_POWER");
    private double  MOVE_MAX_POWER               = propertiesLoader.getDoubleProperty("MOVE_MAX_POWER");

    private double OPEN_LATCH_SERVO_POSITION  = propertiesLoader.getDoubleProperty("OPEN_LATCH_SERVO_POSITION");
    private double CLOSE_LATCH_SERVO_POSITION = propertiesLoader.getDoubleProperty("CLOSE_LATCH_SERVO_POSITION");

    private double BLOCK_FORWARD_OFF_WALL_TO_BLOCK = propertiesLoader.getDoubleProperty("BLOCK_FORWARD_OFF_WALL_TO_BLOCK");
    private double BLOCK_FORWARD_LITTLE_OFF_WALL   = propertiesLoader.getDoubleProperty("BLOCK_FORWARD_LITTLE_OFF_WALL");

    private double BLOCK_ONE_STRAFE_TO_BLOCK   = propertiesLoader.getDoubleProperty("BLOCK_ONE_STRAFE_TO_BLOCK");
    private double BLOCK_TWO_STRAFE_TO_BLOCK   = propertiesLoader.getDoubleProperty("BLOCK_TWO_STRAFE_TO_BLOCK");
    private double BLOCK_THREE_STRAFE_TO_BLOCK = propertiesLoader.getDoubleProperty("BLOCK_THREE_STRAFE_TO_BLOCK");

    private double BLOCK_ONE_FORWARD_TO_FOUNDATION   = propertiesLoader.getDoubleProperty("BLOCK_ONE_FORWARD_TO_FOUNDATION");
    private double BLOCK_TWO_FORWARD_TO_FOUNDATION   = propertiesLoader.getDoubleProperty("BLOCK_TWO_FORWARD_TO_FOUNDATION");
    private double BLOCK_THREE_FORWARD_TO_FOUNDATION = propertiesLoader.getDoubleProperty("BLOCK_THREE_FORWARD_TO_FOUNDATION");

    private double BLOCK_FOUR_BACKWARD_TO_BLOCK = propertiesLoader.getDoubleProperty("BLOCK_FOUR_BACKWARD_TO_BLOCK");
    private double BLOCK_FIVE_BACKWARD_TO_BLOCK = propertiesLoader.getDoubleProperty("BLOCK_FIVE_BACKWARD_TO_BLOCK");
    private double BLOCK_SIX_BACKWARD_TO_BLOCK  = propertiesLoader.getDoubleProperty("BLOCK_SIX_BACKWARD_TO_BLOCK");

    private double BLOCK_FOUR_FORWARD_TO_FOUNDATION = propertiesLoader.getDoubleProperty("BLOCK_FOUR_FORWARD_TO_FOUNDATION");
    private double BLOCK_FIVE_FORWARD_TO_FOUNDATION = propertiesLoader.getDoubleProperty("BLOCK_FIVE_FORWARD_TO_FOUNDATION");
    private double BLOCK_SIX_FORWARD_TO_FOUNDATION  = propertiesLoader.getDoubleProperty("BLOCK_SIX_FORWARD_TO_FOUNDATION");

    private double BLOCK_ANGLE_SUCK_BLOCK       = propertiesLoader.getDoubleProperty("BLOCK_ANGLE_SUCK_BLOCK");
    private double BLOCK_FORWARD_SUCK_UP        = propertiesLoader.getDoubleProperty("BLOCK_FORWARD_SUCK_UP");
    private double BLOCK_FORWARD_SUCK_MIN_POWER = propertiesLoader.getDoubleProperty("BLOCK_FORWARD_SUCK_MIN_POWER");
    private double BLOCK_FORWARD_SUCK_MAX_POWER = propertiesLoader.getDoubleProperty("BLOCK_FORWARD_SUCK_MAX_POWER");
    private double BLOCK_BACKWARD_SUCK_UP       = propertiesLoader.getDoubleProperty("BLOCK_BACKWARD_SUCK_UP");

    private double BLOCK_ARM_UP_ENCODER_POSITION   = propertiesLoader.getDoubleProperty("BLOCK_ARM_UP_ENCODER_POSITION");
    private double BLOCK_ARM_DOWN_ENCODER_POSITION = propertiesLoader.getDoubleProperty("BLOCK_ARM_DOWN_ENCODER_POSITION");

    private double BLOCK_BACKWARD_TO_PARK                     = propertiesLoader.getDoubleProperty("BLOCK_BACKWARD_TO_PARK");
    private double BLOCK_EXTRA_DISTANCE_HORIZONTAL_FOUNTATION = propertiesLoader.getDoubleProperty("BLOCK_EXTRA_DISTANCE_HORIZONTAL_FOUNTATION");


    private double horizontalTurnDegree = 90;
    private double verticalTurnDegree   = 90;
    private double fiddleDistance       = - 3;

    private ElapsedTime runtime = new ElapsedTime();

    private RobotLinearOpMode robot;

    private FOUNDATION_ORIENTATION foundationOrientation;
    private PARK_POSITION          parkPosition;
    private SIDE                   side;
    private ALLIANCE               alliance;
    private LinearOpMode           opMode;
    private SKYSTONE_POSITION      skystone_position;

    private LinkedBlockingQueue<Runnable> intakeAndArmThread = new LinkedBlockingQueue<>();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid   = - 1;
    private static int valLeft  = - 1;
    private static int valRight = - 1;

    private static float[] midPos   = new float[2];
    private static float[] leftPos  = new float[2];
    private static float[] rightPos = new float[2];
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private Thread thread = new Thread(() -> {

        while (true) {

            try {
                intakeAndArmThread.take().run();
            } catch (InterruptedException ignored) {

            }

        }

    });

    public DWAIAutonomous(FOUNDATION_ORIENTATION foundationOrientation, PARK_POSITION parkPosition, SIDE side, ALLIANCE alliance, LinearOpMode opMode) {
        this.foundationOrientation = foundationOrientation;
        this.parkPosition          = parkPosition;
        this.side                  = side;
        this.alliance              = alliance;
        this.opMode                = opMode;
    }

    public void runOpMode() {
        robot = new RobotLinearOpMode(opMode);

        //MAYBE PUT INTO CONSTRUCTOR

        setupVariables();

        if (side == SIDE.BLOCK) {
            openCVinit();
        }

        opMode.waitForStart();
        runtime.reset();

        if (side == SIDE.FOUNDATION) {
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

        }
        else if (side == SIDE.BLOCK) {
            thread.start();

            getSkyStonePosition();

            robot.moveByInches2(BLOCK_FORWARD_LITTLE_OFF_WALL, FORWARD);
            deploy();

            strafeToBlock();
            robot.moveByInches2(BLOCK_FORWARD_OFF_WALL_TO_BLOCK, FORWARD);

            grabBlockFirstTime();

            forwardToFoundationFirstTime();


            dropOffBlock();

            backwardToBlocks();


            grabBlockSecondTime();

            forwardToFoundationSecondTime();

            dropOffBlock();

            blockPark();
        }

        AutoTransitioner.transitionOnStop(opMode, "Skystone Main Teleop");

    }

    private void getSkyStonePosition() {
        if (alliance == ALLIANCE.RED) {
            if (valLeft == 0)
                skystone_position = SKYSTONE_POSITION.THREE_AND_SIX;
            if (valMid == 0)
                skystone_position = SKYSTONE_POSITION.TWO_AND_FIVE;
            if (valRight == 0)
                skystone_position = SKYSTONE_POSITION.ONE_AND_FOUR;
        }
        else {
            if (valLeft == 0)
                skystone_position = SKYSTONE_POSITION.ONE_AND_FOUR;
            if (valMid == 0)
                skystone_position = SKYSTONE_POSITION.TWO_AND_FIVE;
            if (valRight == 0)
                skystone_position = SKYSTONE_POSITION.THREE_AND_SIX;
        }

        if (skystone_position == null) {
            throw new RuntimeException("No skystone detected!");
        }
    }

    private void setupVariables() {
        //Necessary since all variables tuned to blue side

        if (alliance == ALLIANCE.RED && side == SIDE.FOUNDATION) {
            DRIVETRAIN_DISTANCE_RIGHT_TO_GET_FOUNDATION *= - 1;
            horizontalTurnDegree *= - 1;
            fiddleDistance *= - 1;
            HWALL_PARK_STRAFE_DISTANCE *= - 1;
            HBRIDGE_PARK_STRAFE_DISTANCE *= - 1;

            verticalTurnDegree *= - 1;
            DRIVETRAIN_DISTANCE_LEFT_TO_CLEAR_FOUNDATION *= - 1;
            VWALL_PARK_STRAFE_DISTANCE *= - 1;
            VBRIDGE_PARK_STRAFE_DISTANCE *= - 1;
        }

    }

    private void moveToFoundation() {
        print("Moving towards foundation");
        robot.moveByInches(DRIVETRAIN_DISTANCE_BACKWARD_TO_GET_OFF_WALL, FORWARD);
        robot.moveByInches(DRIVETRAIN_DISTANCE_RIGHT_TO_GET_FOUNDATION, STRAFE);
        robot.moveByInchesMaxPower(DRIVETRAIN_DISTANCE_BACKWARD_TO_GET_FOUNDATION, FORWARD, MOVE_MAX_POWER);
    }

    private void latchFoundation() {
        print("Latching foundation");
        robot.setLatchPosition(OPEN_LATCH_SERVO_POSITION);

        while (! robot.getFoundationSensorPressed()) {
            robot.mecanumPowerDrive(0, - .3, 0);
        }

        robot.setLatchPosition(CLOSE_LATCH_SERVO_POSITION);
        robot.moveByInchesMaxPower(- 2, FORWARD, .6);
    }

    private void placeHorizontal() {
        print("Driving forwards");
        robot.moveByInches(DRIVETRAIN_DISTANCE_FORWARD_TO_TURN, FORWARD);

        print("Turning to be forward");

        robot.turnToDegree(horizontalTurnDegree);
        robot.setLatchPosition(OPEN_LATCH_SERVO_POSITION);
    }

    private void parkHorizontal() {
        print("Fiddling with latch");
        robot.moveByInches(fiddleDistance, STRAFE);

        if (parkPosition == PARK_POSITION.WALL) {
            print("Strafing against wall");
            robot.moveByInches(HWALL_PARK_STRAFE_DISTANCE, STRAFE);
        }
        else {
            print("Strafing towards bridge");
            robot.moveByInches(HBRIDGE_PARK_STRAFE_DISTANCE, STRAFE);
        }

        print("Deploying intake");
        deploy();

        print("Moving under bridge");
        robot.moveByInches(36, FORWARD);
        //rough movement, likely replace with color sensor line code
    }

    private void placeVertical() {
        print("Driving forwards");
        robot.moveByInches(DRIVETRAIN_DISTANCE_FORWARD_TO_DEPOT, FORWARD);

        print("Opening latch");
        robot.setLatchPosition(OPEN_LATCH_SERVO_POSITION);

        print("Strafing away from the platform");
        robot.moveByInchesMaxPower(DRIVETRAIN_DISTANCE_LEFT_TO_CLEAR_FOUNDATION, STRAFE, STRAFE_MAX_POWER);

        print("Moving up parallel with platform");
        robot.moveByInches(DRIVETRAIN_DISTANCE_BACKWARD_TO_MIDDLE_OF_FOUNDATION, FORWARD);

        print("Turning to be forward");

        robot.turnToDegree(verticalTurnDegree);

        print("Slamming foundation against the wall really really hard");
        robot.moveByInches(DRIVETRAIN_DISTANCE_BACKWARD_TO_SLAM_FOUNDATION_REALLY_REALLY_HARD, FORWARD);
    }

    private void parkVertical() {

        if (parkPosition == PARK_POSITION.WALL) {
            print("Strafing against wall");
            robot.moveByInches(VWALL_PARK_STRAFE_DISTANCE, STRAFE);
        }
        else {
            print("Strafing against bridge");
            robot.moveByInches(VBRIDGE_PARK_STRAFE_DISTANCE, STRAFE);
        }

        print("Deploying intake");
        deploy();

        print("Moving under bridge");
        robot.moveByInches(22, FORWARD);
    }

    private void print(String printString) {
        opMode.telemetry.addLine(printString);
        opMode.telemetry.update();

        if (PAUSE_STEPS) {
            robot.stopAllMotors();
            //Ensures button is pressed and released before continuing
            while (opMode.opModeIsActive() && ! opMode.gamepad1.a) {
            }
            while (opMode.opModeIsActive() && opMode.gamepad1.a) {
            }
        }

    }

    public enum FOUNDATION_ORIENTATION {
        HORIZONTAL, VERTICAL
    }

    public enum PARK_POSITION {
        WALL, BRIDGE
    }

    public enum ALLIANCE {
        RED, BLUE
    }

    public enum SIDE {
        BLOCK, FOUNDATION
    }

    private void blockPark() {
        robot.moveByInches2(
                BLOCK_BACKWARD_TO_PARK - BLOCK_EXTRA_DISTANCE_HORIZONTAL_FOUNTATION, FORWARD);
    }


    private void deploy() {
        robot.setLiftPower(- 0.2);
        opMode.sleep(1000);
        robot.setLiftPower(0);
        robot.setArmPower(.2);
        opMode.sleep(750);
        robot.setArmPower(0);

        robot.setLiftZeroPowerProperty(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.setArmZeroPowerProperty(DcMotor.ZeroPowerBehavior.FLOAT);
        opMode.sleep(1000);
        robot.setLiftRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setArmRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.setLiftRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setArmRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void strafeToBlock() {
        double strafeDistance = 0;

        //strafing to in front of the block
        switch (skystone_position) {
            case ONE_AND_FOUR:
                strafeDistance = BLOCK_ONE_STRAFE_TO_BLOCK;
                break;
            case TWO_AND_FIVE:
                strafeDistance = BLOCK_TWO_STRAFE_TO_BLOCK;
                break;
            case THREE_AND_SIX:
                strafeDistance = BLOCK_THREE_STRAFE_TO_BLOCK;
                break;
        }

        robot.moveByInches2(strafeDistance, STRAFE);
    }

    private void forwardToFoundationFirstTime() {
        robot.setIntakePower(- 1);

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

        if (foundationOrientation == FOUNDATION_ORIENTATION.HORIZONTAL) {
            forwardDistance += BLOCK_EXTRA_DISTANCE_HORIZONTAL_FOUNTATION;
        }

        robot.moveByInches2(forwardDistance, FORWARD);

        robot.setIntakePower(0);
    }

    private void backwardToBlocks() {
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

        if (foundationOrientation == FOUNDATION_ORIENTATION.HORIZONTAL) {
            backwardDistance -= BLOCK_EXTRA_DISTANCE_HORIZONTAL_FOUNTATION;
        }

        robot.moveByInches2(backwardDistance, FORWARD);
    }

    private void forwardToFoundationSecondTime() {
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

        robot.moveByInches(forwardDistance, FORWARD);

        robot.setIntakePower(0);
    }

    private void grabBlockFirstTime() {
        robot.turnToDegree(BLOCK_ANGLE_SUCK_BLOCK);
        robot.openGrabber();
        robot.setIntakePower(- 1);

//        intakeAndArmThread.add(() -> {
//
//            while (! robot.getBlockSensorPressed()) {
//            }
//
//            robot.setIntakePower(0);
//
//            try {
//                Thread.sleep(250);
//            } catch (InterruptedException ignored) {
//
//            }
//
//            robot.closeGrabber();
//            robot.threadedMoveArmByEncoder(500);
//
//        });

        robot.moveByInches(BLOCK_FORWARD_SUCK_UP, FORWARD, BLOCK_FORWARD_SUCK_MIN_POWER, BLOCK_FORWARD_SUCK_MAX_POWER);
        robot.moveByInches2(BLOCK_BACKWARD_SUCK_UP, FORWARD);
        robot.turnToDegree(90);
        robot.closeGrabber();
        robot.threadedMoveArmByEncoder(500);


    }

    private void grabBlockSecondTime() {
        robot.turnToDegree(BLOCK_ANGLE_SUCK_BLOCK);
        robot.openGrabber();
        robot.setIntakePower(- 1);

        intakeAndArmThread.add(() -> {

            while (! robot.getBlockSensorPressed()) {
            }

            robot.setIntakePower(0);
            robot.closeGrabber();
            robot.threadedMoveArmByEncoder(500);
        });

        robot.moveByInches(BLOCK_FORWARD_SUCK_UP, FORWARD, BLOCK_FORWARD_SUCK_MIN_POWER, BLOCK_FORWARD_SUCK_MAX_POWER);
        robot.moveByInches(BLOCK_BACKWARD_SUCK_UP, FORWARD);
        robot.turnToDegree(90);
    }

    private void dropOffBlock() {
        robot.setIntakePower(0);
        robot.closeGrabber();
        robot.moveArmByEncoder(BLOCK_ARM_UP_ENCODER_POSITION);
        robot.setIntakePower(.5);
        robot.openGrabber();
        robot.moveArmByEncoder(BLOCK_ARM_DOWN_ENCODER_POSITION);
        robot.setIntakePower(0);
    }

    enum SKYSTONE_POSITION {
        ONE_AND_FOUR, TWO_AND_FIVE, THREE_AND_SIX
    }

    private void openCVinit() {
        float offsetX   = propertiesLoader.getFloatProperty("OFFSET_X");//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
        float offsetY   = propertiesLoader.getFloatProperty("OFFSET_Y");//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive
        float distScale = propertiesLoader.getFloatProperty("DIST_SCALE");

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

    //detection pipeline (ignore)
    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat              yCbCrChan2Mat = new Mat();
        Mat              thresholdMat  = new Mat();
        Mat              all           = new Mat();
        List<MatOfPoint> contoursList  = new ArrayList<>();

        enum Stage {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private StageSwitchingPipeline.Stage stageToRenderToViewport;

        {
            stageToRenderToViewport = StageSwitchingPipeline.Stage.detection;
        }

        private StageSwitchingPipeline.Stage[] stages = StageSwitchingPipeline.Stage.values();

        @Override
        public void onViewportTapped() {
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
        public Mat processFrame(Mat input) {
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
            double[] pixMid = thresholdMat.get((int) (input.rows() * midPos[1]), (int) (input.cols()
                                                                                        * midPos[0]));//gets value at circle
            valMid = (int) pixMid[0];
            //valMid = radialAverage(midPos, input);

            double[] pixLeft = thresholdMat.get((int) (input.rows() * leftPos[1]), (int) (
                    input.cols() * leftPos[0]));//gets value at circle
            valLeft = (int) pixLeft[0];
            //valMid = radialAverage(leftPos, input);

            double[] pixRight = thresholdMat.get((int) (input.rows() * rightPos[1]), (int) (
                    input.cols() * rightPos[0]));//gets value at circle
            valRight = (int) pixRight[0];
            //valRight = radialAverage(rightPos, input);

            //create three points
            Point pointMid   = new Point((int) (input.cols() * midPos[0]), (int) (input.rows()
                                                                                  * midPos[1]));
            Point pointLeft  = new Point((int) (input.cols() * leftPos[0]), (int) (input.rows()
                                                                                   * leftPos[1]));
            Point pointRight = new Point((int) (input.cols() * rightPos[0]), (int) (input.rows()
                                                                                    * rightPos[1]));

            //draw circles on those points
            int radius = 5;
            Imgproc.circle(all, pointMid, radius, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointLeft, radius, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointRight, radius, new Scalar(255, 0, 0), 1);//draws circle

            //draw 3 rectangles
            float rectHeight = .6f / 8f;
            float rectWidth  = 1.5f / 8f;
            Imgproc.rectangle(//1-3
                    all, new Point(
                            input.cols() * (leftPos[0] - rectWidth / 2),
                            input.rows() * (leftPos[1] - rectHeight / 2)), new Point(
                            input.cols() * (leftPos[0] + rectWidth / 2), input.rows() * (leftPos[1]
                                                                                         +
                                                                                         rectHeight
                                                                                         / 2)), new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all, new Point(
                            input.cols() * (midPos[0] - rectWidth / 2),
                            input.rows() * (midPos[1] - rectHeight / 2)), new Point(
                            input.cols() * (midPos[0] + rectWidth / 2),
                            input.rows() * (midPos[1] + rectHeight / 2)), new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all, new Point(
                            input.cols() * (rightPos[0] - rectWidth / 2),
                            input.rows() * (rightPos[1] - rectHeight / 2)), new Point(
                            input.cols() * (rightPos[0] + rectWidth / 2),
                            input.rows() * (rightPos[1]
                                            + rectHeight / 2)), new Scalar(0, 255, 0), 3);

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

    }
}
