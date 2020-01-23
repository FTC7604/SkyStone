package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.IO.PropertiesLoader;
import org.firstinspires.ftc.teamcode.Robot.*;
import org.firstinspires.ftc.teamcode.IO.RuntimeLogger;

import java.util.*;

/**  TEST COLOR SENSORS + LOGGING  */

@TeleOp(name="Block Color Sensing Test", group="TeleOp")
@Disabled
public class BlockColorSensingTest extends LinearOpMode {
    RobotLinearOpMode robot;
    PropertiesLoader propertiesLoader = new PropertiesLoader("Autonomous");
    double BLOCK_POWER = propertiesLoader.getDoubleProperty("BLOCK_POWER");
    int NUM_VALS = propertiesLoader.getIntegerProperty("NUM_VALS");
    double DIST_THRESHOLD = propertiesLoader.getDoubleProperty("DIST_THRESHOLD");
    int DETECT_COUNT = propertiesLoader.getIntegerProperty("DETECT_COUNT");
    double SMOOTH_RATIO = propertiesLoader.getDoubleProperty("SMOOTH_RATIO");
    RuntimeLogger logger = new RuntimeLogger("colorVals");
    int logCount = 0;

    double DIST_SCALE_FACTOR = propertiesLoader.getDoubleProperty("DIST_SCALE_FACTOR");
    double RBLOCK_THRESHOLD = propertiesLoader.getDoubleProperty("RBLOCK_THRESHOLD");

    @Override
    public void runOpMode(){
        LinkedList<Double> distances = new LinkedList();
        int detected = 0;
        double[] colors;
        double minDist = 0;
        double dist = 0;
        double prevDist;
        double prevChange = 0;
        double currentChange = 0;
        double smoothChange = 0;

        robot = new RobotLinearOpMode(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        telemetry.clearAll();
        double newValue = scaleArray(robot.getColors())[0];
        robot.mecanumPowerDrive(0, BLOCK_POWER, 0);

        while(!isStopRequested() && detected < DETECT_COUNT){
            prevDist = dist;
            dist = robot.getDistance();
            colors = robot.getColors();

            if(prevDist != dist || (logCount % 2 == 0 && Double.isNaN(dist))) {
                logger.write("D: " + dist + " R: " + colors[0] + " G: " + colors[1] + " B: " + colors[2]);

                if(Double.isNaN(dist)) {
                    logCount++;
                }

            }

            if(dist > 0 && dist != prevDist) {
                distances.add(dist);
                while(distances.size() > NUM_VALS){distances.poll();}

                //DISTANCE CODE

                if(distances.size() == NUM_VALS){
                    double compDistance = distances.pollLast();
                    double avg = averageQueue(distances, 1);
                    currentChange = Math.abs((avg - compDistance) / compDistance);
                    smoothChange = SMOOTH_RATIO * currentChange + (1 - SMOOTH_RATIO) * prevChange;
                    prevChange = currentChange;

                    if(smoothChange > DIST_THRESHOLD){
                        telemetry.addData("Status", "Detected");
                        detected++;

                        if(detected >= DETECT_COUNT) {
                            robot.mecanumPowerDrive(0, 0, 0);
                        }

                        //logger.write("Percent Change: " + percentChange + " Average Distance: " + avg
                        //        + " Current Distance: " + compDistance + " DETECTED " + "(" + detected + ")");
                    } else{
                        telemetry.addData("Status", "Not Detected");
                        robot.mecanumPowerDrive(0, BLOCK_POWER, 0);

                        //logger.write("Percent Change: " + percentChange + " Average Distance: " + avg
                        //        + " Current Distance: " + compDistance);
                    }

                    telemetry.addData("Smooth change", smoothChange);
                    telemetry.addData("Avg. distance", avg);
                } else if(distances.size() < NUM_VALS){
                    telemetry.addData("Status", "Not Detected");
                    robot.mecanumPowerDrive(0, BLOCK_POWER, 0);

                    //logger.write("Current Distance: " + distances.peekLast());
                }

                //logger.write(toString(distances));
            }

            //COLOR CODE

            /*while(distances.size() > 5){
                distances.poll();
            }

            colors = scaleArray(robot.getColors(COLOR_SENSOR.LEFT));
            minDist = minVal(distances);
            newValue = 0.3 * newValue + 0.7 * colors[0];
            telemetry.addData("Distance", robot.getDistance(COLOR_SENSOR.LEFT));
            telemetry.addData("Smooth R", newValue);

            if(newValue <= RBLOCK_THRESHOLD + DIST_SCALE_FACTOR / minDist){
                telemetry.addData("Status", "Detected");
                robot.mecanumPowerDrive(0, 0, 0);
                //detected = true;
            } else{
                telemetry.addData("Status", "Not Detected");
                robot.mecanumPowerDrive(0, BLOCK_POWER, 0);
            }

            telemetry.addData("Augmented Threshold", RBLOCK_THRESHOLD + DIST_SCALE_FACTOR / minDist);*/
            telemetry.update();
        }

        robot.mecanumPowerDrive(0, 0, 0);
        telemetry.clearAll();
        telemetry.addData("Status", "Finished");
        telemetry.update();
        logger.close();
        sleep(2000);
    }

    public String toString(Queue q){
        Object[] vals = q.toArray();
        String n = "";

        for(int i = 0; i < vals.length; i++){

            if(vals[i] != null){
                n += i + ": " + vals[i] + " ";
            }

        }

        return n;
    }

    public double minVal(Queue q){
        Object[] vals = q.toArray();
        double min = 0;

        for(int i = 0; i < vals.length; i++){

            if(vals[i] != null && (double) vals[i] != 0) {

                if(min == 0 || (double) vals[i] < min) {
                    min = (double) vals[i];
                }

            }

        }

        return min;
    }

    public double averageQueue(Queue q, int offset){
        Object[] vals = q.toArray();
        double avg = 0;
        double actualSize = vals.length - offset;

        for(int i = 0; i < vals.length - offset; i++){

            if(vals[i] != null) {
                avg += (double) vals[i];
            } else{
                actualSize -= 1;
            }

        }

        return avg / actualSize;
    }

    public double[] scaleArray(double[] array){
        double max = 0;

        for(int i = 0; i < array.length; i++){

            if(array[i] > max){
                max = array[i];
            }

        }

        for(int i = 0; i < array.length; i++){
            array[i] = array[i] / max;
        }

        return array;
    }

}
