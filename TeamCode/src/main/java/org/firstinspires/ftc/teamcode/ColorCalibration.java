package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.Robot.*;

@TeleOp(name="Color Test", group="TeleOp")
public class ColorCalibration extends LinearOpMode {
    RobotLinearOpMode robot;

    PropertiesLoader propertiesLoader = new PropertiesLoader("Autonomous");
    double RBLOCK_THRESHOLD = propertiesLoader.getDoubleProperty("RBLOCK_THRESHOLD");
    double BLOCK_POWER = propertiesLoader.getDoubleProperty("BLOCK_POWER");

    @Override
    public void runOpMode(){
        robot = new RobotLinearOpMode(this);
        double[] colors;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        telemetry.clear();
        RBLOCK_THRESHOLD = scaleArray(robot.getColors())[0];

        double currentDistance;
        double prevDistance = robot.getDistance();

        while(!isStopRequested()){
            colors = scaleArray(robot.getColors());
            currentDistance = robot.getDistance();

            /*if(colors[0] <= RBLOCK_THRESHOLD * 0.95){
                telemetry.addData("Status", "Detected");
                robot.mecanumPowerDrive(0, 0, 0);
            } else{
                telemetry.addData("Status", "Not Detected");
                robot.mecanumPowerDrive(0, BLOCK_POWER, 0);
            }*/

            double percentChange = Math.abs((currentDistance - prevDistance) / prevDistance);

            if(percentChange > 0.5){
                telemetry.addData("Status", "Detected");
                robot.mecanumPowerDrive(0, 0, 0);
            } else{
                telemetry.addData("Status", "Not Detected");
                robot.mecanumPowerDrive(0, BLOCK_POWER, 0);
            }

            telemetry.addData("Red", colors[0]);
            telemetry.addData("Green", colors[1]);
            telemetry.addData("Blue", colors[2]);
            telemetry.addData("Distance", robot.getDistance());
            telemetry.addData("Percent Change", percentChange);
            telemetry.update();

            prevDistance = currentDistance;
        }

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
