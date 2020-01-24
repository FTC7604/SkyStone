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
    private final double topLimit;
    private final double bottomLimit;
    private final boolean has_limits;

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
        this.has_limits            = false;
        this.topLimit              = 0;
        this.bottomLimit           = 0;

    }

    public BetterBalisticProfile(
            final double acceleration_distance,
            final double deceleration_distance,
            final double start_power,
            final double full_power,
            final double end_power,
            final double first_limit,
            final double second_limit,
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
        this.has_limits            = true;

        if (first_limit > second_limit) {
            this.topLimit    = first_limit;
            this.bottomLimit = second_limit;
        }
        else {
            this.topLimit    = second_limit;
            this.bottomLimit = first_limit;
        }

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
        if (movementIsBackward) return -1;
        else return 1;
    }

    private double accelerationCurve() {
        if (!tempPowerSet) {
            return processedCurve((full_power - start_power) * invert(), (currentPosition - startPosition) * invert(), acceleration_distance, acceleration_curve, start_power * invert());
        } else {
            return processedCurve((full_power - tempStartPower) * invert(), (currentPosition - startPosition) * invert(), acceleration_distance, acceleration_curve, tempStartPower * invert());
        }
    }

    private double decelerationCurve(){
        if(!tempPowerSet) {
            return processedCurve((full_power - end_power) * invert(), (endPosition - currentPosition) * invert(), deceleration_distance, deceleration_curve, end_power * invert());


        } else  {
            return processedCurve((full_power - tempEndPower) * invert(), (endPosition - currentPosition) * invert(), deceleration_distance, deceleration_curve, tempEndPower * invert());

        }
    }

    public void setCurve(double startPosition, double endPosition){

        if (has_limits) {
            if (this.startPosition > topLimit) this.startPosition = topLimit;
            else if (this.startPosition < bottomLimit) this.startPosition = bottomLimit;
            else this.startPosition = startPosition;

            if (this.endPosition > topLimit) this.endPosition = topLimit;
            else if (this.endPosition < bottomLimit) this.endPosition = bottomLimit;
            else this.endPosition = endPosition;
        } else {
            this.startPosition = startPosition;
            this.endPosition = endPosition;
        }

        this.movementIsBackward = startPosition > endPosition;
        this.currentPosition    = startPosition;
    }

    double tempStartPower;
    double tempEndPower;
    boolean tempPowerSet = false;

    public void setCurve(double startPosition, double endPosition, double tempMinPower, double tempMaxPower){
        this.tempStartPower = tempMinPower;
        this.tempEndPower = tempMaxPower;
        tempPowerSet = true;
        setCurve(startPosition, endPosition);
    }

    public void endCurve(){
        tempPowerSet = false;
        this.tempStartPower = 0;
        this.tempEndPower = 0;
    }

    public void setCurrentPosition(double currentPosition){
        this.currentPosition = currentPosition;
        isAccelerating       = abs(startPosition - currentPosition) < acceleration_distance;
        isDecelerating       = abs(endPosition - currentPosition) < deceleration_distance;
    }

    public void getRidOfTemp(){
        tempPowerSet = false;
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

    public boolean isBackwards(){return movementIsBackward;}

    public boolean isDone(){
        //return currentPosition * invert() > endPosition;
        //return abs(currentPosition - endPosition) < deceleration_distance / 25;

        if(movementIsBackward){
            return currentPosition <= endPosition + deceleration_distance / 100 && currentPosition >= endPosition - deceleration_distance;
        } else{
            return endPosition <= currentPosition + deceleration_distance / 100 && endPosition >= currentPosition - deceleration_distance;
        }

    }

    public enum CURVE_TYPE {
        LINEAR,
        SINUSOIDAL_NORMAL,
        SINUSOIDAL_INVERTED,
        SINUSOIDAL_SCURVE
    }

}
