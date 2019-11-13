package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.Robot.*;
import java.util.*;

@TeleOp(name="Color Test", group="TeleOp")
public class ColorCalibration extends LinearOpMode {
    RobotLinearOpMode robot;

    PropertiesLoader propertiesLoader = new PropertiesLoader("Autonomous");
    double RBLOCK_THRESHOLD = propertiesLoader.getDoubleProperty("RBLOCK_THRESHOLD");
    double BLOCK_POWER = propertiesLoader.getDoubleProperty("BLOCK_POWER");
    int NUM_VALS = propertiesLoader.getIntegerProperty("NUM_VALS");
    double DIST_THRESHOLD = propertiesLoader.getDoubleProperty("DIST_THRESHOLD");
    double DIST_SCALE_FACTOR = propertiesLoader.getDoubleProperty("DIST_SCALE_FACTOR");
    int DETECT_COUNT = propertiesLoader.getIntegerProperty("DETECT_COUNT");

    @Override
    public void runOpMode(){
        LinkedList<Double> distances = new LinkedList();
        int detected = 0;

        robot = new RobotLinearOpMode(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        telemetry.clearAll();
        double[] colors;
        double minDist = 0;
        double newValue = scaleArray(robot.getColors(COLOR_SENSOR.LEFT))[0];
        double dist;

        while(!isStopRequested() && detected < DETECT_COUNT){
            dist = robot.getDistance(COLOR_SENSOR.LEFT);

            if(dist > 0) {
                distances.add(dist);
            }

            //DISTANCE CODE

            if(distances.size() == NUM_VALS){
                double compDistance = distances.poll();
                double avg = averageQueue(distances);
                double percentChange = Math.abs((avg - compDistance) / compDistance);

                if(percentChange > DIST_THRESHOLD){
                    telemetry.addData("Status", "Detected");
                    detected++;

                    if(detected >= DETECT_COUNT) {
                        robot.mecanumPowerDrive(0, 0, 0);
                    }

                } else{
                    telemetry.addData("Status", "Not Detected");
                    robot.mecanumPowerDrive(0, BLOCK_POWER, 0);
                }

                telemetry.addData("Percent change", percentChange);
                telemetry.addData("Avg. distance", avg);
            } else if(distances.size() < NUM_VALS){
                telemetry.addData("Status", "Not Detected");
                robot.mecanumPowerDrive(0, BLOCK_POWER, 0);
            } else{

                while(distances.size() > NUM_VALS){distances.poll();}

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

        telemetry.clearAll();
        telemetry.addData("Status", "Finished");
        telemetry.update();

        sleep(2000);
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

    public double averageQueue(Queue q){
        Object[] vals = q.toArray();
        double avg = 0;
        double actualSize = vals.length;

        for(int i = 0; i < vals.length; i++){

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
