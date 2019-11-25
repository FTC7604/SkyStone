package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "I am Sensor", group = "Linear Opmode")
public class SensorTest extends LinearOpMode {
    DistanceSensor d1;
    DistanceSensor d2;
    DistanceSensor d3;

    ColorSensor c1;
    ColorSensor c2;
    ColorSensor c3;

    private void setupHardware(){
        c1 = hardwareMap.get(ColorSensor.class, "c1");
        d1 = hardwareMap.get(DistanceSensor.class, "c1");
        c2 = hardwareMap.get(ColorSensor.class, "c2");
        d2 = hardwareMap.get(DistanceSensor.class, "c2");
        c3 = hardwareMap.get(ColorSensor.class, "c3");  //The V2
        d3 = hardwareMap.get(DistanceSensor.class, "c3");
    }

    @Override
    public void runOpMode(){
        setupHardware();

        double dist1;
        double dist2;
        double dist3;

        double[] color1;
        double[] color2;
        double[] color3;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.clear();

        while(!isStopRequested()){
            dist1 = getDistance(d1);
            dist2 = getDistance(d2);
            dist3 = getDistance(d3);

            color1 = getColors(c1);
            color2 = getColors(c2);
            color3 = getColors(c3);

            telemetry.addLine("PORT 1: D: " + dist1 + " R: " + color1[0]);
            telemetry.addLine("PORT 2: D: " + dist2 + " R: " + color2[0]);
            telemetry.addLine("PORT 3: D: " + dist3 + " R: " + color3[0]);
            telemetry.update();
        }

    }

    private double[] getColors(ColorSensor c){
        double[] colors = new double[3];
        colors[0] = c.red();
        colors[1] = c.green();
        colors[2] = c.blue();
        return colors;
    }

    private double getDistance(DistanceSensor d){
        return d.getDistance(DistanceUnit.MM);
    }

}
