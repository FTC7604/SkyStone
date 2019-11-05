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

    @Override
    public void runOpMode(){
        LinkedList<Double> distances = new LinkedList();
        Boolean detected = false;

        robot = new RobotLinearOpMode(this);
        double[] colors;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        telemetry.clear();
        RBLOCK_THRESHOLD = scaleArray(robot.getColors())[0];

        while(!isStopRequested() && !detected){
            colors = scaleArray(robot.getColors());
            distances.add(robot.getDistance());

            if(distances.size() == 6){
                double compDistance = distances.poll();
                double avg = averageQueue(distances);
                double percentChange = Math.abs((avg - compDistance) / compDistance);

                if(percentChange > 0.2){
                    telemetry.addData("Status", "Detected");
                    robot.mecanumPowerDrive(0, 0, 0);
                    detected = true;
                } else{
                    telemetry.addData("Status", "Not Detected");
                    robot.mecanumPowerDrive(0, BLOCK_POWER, 0);
                }

                telemetry.addData("Percent change", percentChange);
                telemetry.addData("Avg. distance", avg);
                telemetry.addData("Distance", robot.getDistance());
            } else{
                telemetry.addData("Status", "Not Detected");
                robot.mecanumPowerDrive(0, BLOCK_POWER, 0);
            }

            /*if(colors[0] <= RBLOCK_THRESHOLD * 0.95){
                telemetry.addData("Status", "Detected");
                robot.mecanumPowerDrive(0, 0, 0);
            } else{
                telemetry.addData("Status", "Not Detected");
                robot.mecanumPowerDrive(0, BLOCK_POWER, 0);
            }*/

            /*telemetry.addData("Red", colors[0]);
            telemetry.addData("Green", colors[1]);
            telemetry.addData("Blue", colors[2]);*/
            telemetry.update();
        }

        telemetry.clearAll();
        telemetry.addData("Status", "Finished");
        telemetry.update();

        sleep(2000);
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

        telemetry.addData("Vals length", vals.length);
        telemetry.addData("Actual size", actualSize);

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
