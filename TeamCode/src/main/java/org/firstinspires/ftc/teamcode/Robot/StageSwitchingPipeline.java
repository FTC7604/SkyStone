package org.firstinspires.ftc.teamcode.Robot;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//detection pipeline (ignore)
public class StageSwitchingPipeline extends OpenCvPipeline {

    private int valMid = -1;
    private int valLeft = -1;
    private int valRight = -1;

    private float[] midPos = new float[2];
    private float[] leftPos = new float[2];
    private float[] rightPos = new float[2];

    private Mat yCbCrChan2Mat = new Mat();
    private Mat thresholdMat = new Mat();
    private Mat all = new Mat();
    private List<MatOfPoint> contoursList = new ArrayList<>();
    private StageSwitchingPipeline.Stage stageToRenderToViewport;
    private StageSwitchingPipeline.Stage[] stages = StageSwitchingPipeline.Stage.values();

    {
        stageToRenderToViewport = StageSwitchingPipeline.Stage.detection;
    }

    public StageSwitchingPipeline(float[] offsets){
        midPos[0]   = (4f + offsets[0]) / 8f;
        midPos[1]   = (4f + offsets[1]) / 8f;
        leftPos[0]  = (4f + offsets[0] - offsets[2] * 2f) / 8f;
        leftPos[1]  = (4f + offsets[1]) / 8f;
        rightPos[0] = (4f + offsets[0] + offsets[2] * 2f) / 8f;
        rightPos[1] = (4f + offsets[1]) / 8f;
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

    public int[] getPositions(){
        int[] ar = {valLeft, valMid, valRight};
        return ar;
    }

}
