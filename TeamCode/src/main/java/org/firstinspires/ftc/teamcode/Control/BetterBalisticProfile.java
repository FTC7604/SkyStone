package org.firstinspires.ftc.teamcode.Control;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class BetterBalisticProfile {
    private final double acceleration_distance;
    private final double deceleration_distance;
    private final double start_power;
    private final double full_power;
    private final double end_power;
    private final CURVE_TYPE acceleration_curve;
    private final CURVE_TYPE deceleration_curve;
    private double startPosition;
    private double endPosition;
    private double currentPosition;
    private boolean movementIsBackward;
    private boolean isAccelerating;
    private boolean isDecelerating;

    public BetterBalisticProfile(
            final double acceleration_distance,
            final double deceleration_distance,
            final double start_power,
            final double full_power,
            final double end_power,
            final CURVE_TYPE acceleration_curve,
            final CURVE_TYPE deceleration_curve
    ){

        this.acceleration_distance = acceleration_distance;
        this.deceleration_distance = deceleration_distance;
        this.start_power           = start_power;
        this.full_power            = full_power;
        this.end_power             = end_power;
        this.acceleration_curve    = acceleration_curve;
        this.deceleration_curve    = deceleration_curve;

    }

    public double getEnd_power(){
        return end_power;
    }

    private double rawCurve(double x, double d, CURVE_TYPE curve_type){

        switch (curve_type) {
            case LINEAR:
                return x / d;
            case SINUSOIDAL_NORMAL:
                return sin((PI / 2) * (x / d));
            case SINUSOIDAL_INVERTED:
                return -cos((PI / 2) * (x / d)) + 1;
            case SINUSOIDAL_SCURVE:
                return -.5 * cos(PI * (x / d)) + .5;
        }

        return 0;
    }

    private double processedCurve(
            double power_adjuster,
            double x,
            double distance_zone,
            CURVE_TYPE curve_type,
            double min_power
    ){
        return power_adjuster * rawCurve(x, distance_zone, curve_type) + min_power;
    }

    private int invert(){
        if (movementIsBackward)
            return -1;
        else
            return 1;
    }

    private double accelerationCurve(){
        return processedCurve((full_power - start_power) * invert(),
                              (currentPosition - startPosition) * invert(),
                              acceleration_distance,
                              acceleration_curve,
                              start_power * invert());
    }

    private double decelerationCurve(){
        return processedCurve((full_power - end_power) * invert(),
                              (endPosition - currentPosition) * invert(),
                              deceleration_distance,
                              deceleration_curve,
                              end_power * invert());
    }

    public void setCurve(double start_position, double end_position){
        this.startPosition      = start_position;
        this.endPosition        = end_position;
        this.movementIsBackward = start_position > end_position;
        this.currentPosition    = start_position;
    }

    public void setCurrentPosition(double currentPosition){
        this.currentPosition = currentPosition;
        isAccelerating       = abs(startPosition - currentPosition) < acceleration_distance;
        isDecelerating       = abs(endPosition - currentPosition) < deceleration_distance;
    }

    public double getCurrentPowerAccelDecel(){
        if (isAccelerating && isDecelerating) {
            return accelerationCurve() * decelerationCurve() * invert();
        }
        else if (isAccelerating) {
            return accelerationCurve();
        }
        else if (isDecelerating) {
            return decelerationCurve();
        }
        else {
            return full_power * invert();
        }
    }

    public double getCurrentPowerDecel(){
        if (isDecelerating) {
            return decelerationCurve();
        }
        else {
            return full_power * invert();
        }
    }

    public double getCurrentPowerAccel(){
        if (isAccelerating) {
            return accelerationCurve();
        }
        else {
            return full_power * invert();
        }
    }

    public double getCurrentPowerFull(){

        return full_power * invert();

    }

    public boolean isDone(){
        return abs(currentPosition - endPosition) < deceleration_distance / 100;
    }

    public enum CURVE_TYPE {
        LINEAR,
        SINUSOIDAL_NORMAL,
        SINUSOIDAL_INVERTED,
        SINUSOIDAL_SCURVE
    }
}
