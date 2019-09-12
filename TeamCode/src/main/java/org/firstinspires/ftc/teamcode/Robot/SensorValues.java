package org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;

public class SensorValues {

    private double xRotation;
    private double yRotation;
    private double zRotation;

    public double getxRotation() {
        return xRotation;
    }

    public void setxRotation(double xRotation) {
        this.xRotation = xRotation;
    }

    public double getyRotation() {
        return yRotation;
    }

    public void setyRotation(double yRotation) {
        this.yRotation = yRotation;
    }

    public double getzRotation() {
        return zRotation;
    }

    public void setzRotation(double zRotation) {
        this.zRotation = zRotation;
    }

    private ArrayList <Double> sensorValues = new ArrayList <Double>();

}
