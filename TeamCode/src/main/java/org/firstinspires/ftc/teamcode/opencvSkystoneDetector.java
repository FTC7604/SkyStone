package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode;
import org.opencv.core.*;
import org.opencv.imgproc.*;
import org.openftc.easyopencv.*;
import org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode;
import java.util.*;

/**
 * Created by maryjaneb  on 11/13/2016.
 * Edited by Jonathan Zhao
 * monitor: 640 x 480
 */
@TeleOp(name = "opencvSkystoneDetector", group = "TeleOp")
public class opencvSkystoneDetector extends LinearOpMode {

    enum SKYSTONE_POSITION {
        ONE_AND_FOUR,
        TWO_AND_FIVE,
        THREE_AND_SIX
    }

    SKYSTONE_POSITION skystone_position;

    private ElapsedTime runtime = new ElapsedTime();

    private static PropertiesLoader propertiesLoader = new PropertiesLoader("Autonomous");

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static int radius = 5;

    private static float rectHeight = .6f / 8f;
    private static float rectWidth = 1.5f / 8f;

    private static float[] midPos = new float[2];
    private static float[] leftPos = new float[2];
    private static float[] rightPos = new float[2];
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;

    RobotLinearOpMode robot;

    @Override
    public void runOpMode() {

        float offsetX = propertiesLoader.getFloatProperty("OFFSET_X");//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
        float offsetY = propertiesLoader.getFloatProperty("OFFSET_Y");//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive
        float distScale = propertiesLoader.getFloatProperty("DIST_SCALE");

        midPos[0] = (4f + offsetX) / 8f;
        midPos[1] = (4f + offsetY) / 8f;
        leftPos[0] = (4f + offsetX - distScale * 2f) / 8f;
        leftPos[1] = (4f + offsetY) / 8f;
        rightPos[0] = (4f + offsetX + distScale * 2f) / 8f;
        rightPos[1] = (4f + offsetY) / 8f;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        robot = new RobotLinearOpMode(this);

        waitForStart();
        runtime.reset();

        telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
        telemetry.addData("Height", rows);
        telemetry.addData("Width", cols);

        telemetry.addData("Point mid: ", midPos[0] + ", " + midPos[1]);
        telemetry.addData("Point left: ", leftPos[0] + ", " + leftPos[1]);
        telemetry.addData("Point right: ", rightPos[0] + ", " + rightPos[1]);

        telemetry.update();
        sleep(100);

        if (valLeft == 255) skystone_position = SKYSTONE_POSITION.THREE_AND_SIX;
        if (valMid == 255) skystone_position = SKYSTONE_POSITION.TWO_AND_FIVE;
        if (valRight == 255) skystone_position = SKYSTONE_POSITION.ONE_AND_FOUR;

        switch(skystone_position){
            case ONE_AND_FOUR:

                telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
                telemetry.update();
                sleep(100000);

                telemetry.addLine("1 and 4");
                telemetry.update();
                sleep(100000);

                robot.moveByInches(-18, RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD);
                robot.moveByInches(12, RobotLinearOpMode.MOVEMENT_DIRECTION.STRAFE);
                robot.turnByDegree(-270);
                robot.moveByInches(6, RobotLinearOpMode.MOVEMENT_DIRECTION.STRAFE);
                robot.setIntakePower(1.0);
                robot.closeGrabber();
                robot.moveByInches(12, RobotLinearOpMode.MOVEMENT_DIRECTION.STRAFE);
                break;
            case TWO_AND_FIVE:
                telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
                telemetry.update();
                sleep(100000);

                telemetry.addLine("2 and 5");
                telemetry.update();
                sleep(100000);

                robot.moveByInches(-18, RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD);
                robot.moveByInches(12, RobotLinearOpMode.MOVEMENT_DIRECTION.STRAFE);
                grabBlock();
                break;
            case THREE_AND_SIX:
                telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
                telemetry.update();
                sleep(100000);

                telemetry.addLine("3 and 6");
                telemetry.update();
                sleep(100000);

                robot.moveByInches(-18, RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD);
                robot.moveByInches(6, RobotLinearOpMode.MOVEMENT_DIRECTION.STRAFE);
                grabBlock();
                break;
        }


    }


    public void grabBlock(){
        robot.turnByDegree(-270);
        robot.moveByInches(-12, RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD);
        robot.moveByInches(-9, RobotLinearOpMode.MOVEMENT_DIRECTION.STRAFE);
        robot.setIntakePower(1.0);
        robot.closeGrabber();
        robot.moveByInches(12, RobotLinearOpMode.MOVEMENT_DIRECTION.STRAFE);
    }




    //detection pipeline (ignore)
    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        private int radialAverage(float[] pos, Mat input) {
            int sum = 0;

            for (int i = -radius; i <= radius; i++) {

                for (int j = -radius; j <= radius; j++) {
                    sum += thresholdMat.get((int) (input.rows() * pos[1]), (int) (input.cols() * pos[0]))[0];
                }

            }

            return sum / (4 * radius * radius);
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
            Point pointMid = new Point((int) (input.cols() * midPos[0]), (int) (input.rows() * midPos[1]));
            Point pointLeft = new Point((int) (input.cols() * leftPos[0]), (int) (input.rows() * leftPos[1]));
            Point pointRight = new Point((int) (input.cols() * rightPos[0]), (int) (input.rows() * rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid, radius, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointLeft, radius, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointRight, radius, new Scalar(255, 0, 0), 1);//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols() * (leftPos[0] - rectWidth / 2),
                            input.rows() * (leftPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (leftPos[0] + rectWidth / 2),
                            input.rows() * (leftPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols() * (midPos[0] - rectWidth / 2),
                            input.rows() * (midPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (midPos[0] + rectWidth / 2),
                            input.rows() * (midPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols() * (rightPos[0] - rectWidth / 2),
                            input.rows() * (rightPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (rightPos[0] + rectWidth / 2),
                            input.rows() * (rightPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);

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