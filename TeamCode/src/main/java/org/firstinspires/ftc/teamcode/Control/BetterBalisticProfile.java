package org.firstinspires.ftc.teamcode.Control;

import static java.lang.Math.*;

public class BetterBalisticProfile {

    //the state of the balistic profile
    enum STATE{
        ACCELERATING,
        FULL_POWER,
        DECELERATING,
        COMPLETE
    }

    //the current curves that are offered
    public enum CURVE {
        SINISUDAL,
        PROPORTIONAL
    }


    public BetterBalisticProfile(double startingPower, double accelerationDistance, double deccelerationDistance, double maximumPower, double rawStartPosition, double changeInPosition, CURVE curve) {
        this.startingPower = startingPower;
        this.accelerationDistance = accelerationDistance;
        this.deccelerationDistance = deccelerationDistance;
        this.maximumPower = maximumPower;
        this.rawStartPosition = rawStartPosition;
        this.currentPosition = 0;
        this.changeInPosition = changeInPosition;
        this.curve = curve;
        this.state = STATE.ACCELERATING;
    }

    public BetterBalisticProfile(double rawStartPosition, double changeInPosition, CURVE curve){
        new BetterBalisticProfile(
                .05,
                1440 * 10,
                1440 * 10,
                .8,
                rawStartPosition,
                changeInPosition,
                curve
        );
    }

    private double startingPower;
    private double accelerationDistance;
    private double deccelerationDistance;
    private double maximumPower;

    private double rawStartPosition;
    private double currentPosition;
    private double changeInPosition;

    private CURVE curve;
    private STATE state;

    public boolean isComplete(){
        return state == STATE.COMPLETE;
    }
    public STATE getState(){return state;}

    public void update(double rawCurrentPosition){
        this.currentPosition = rawCurrentPosition - rawStartPosition;
        updateState();
    }


    //this method updates the state of the curve based on the position of the system in its curve
    private void updateState(){
        //checks to see if the system has reached the end of the loop
        if(abs(currentPosition)>= abs(changeInPosition)){
            state = STATE.COMPLETE;
        }
        //if it hasn't gotten farther than the accelerationDistance, then it is accelerating
        if(abs(currentPosition) < abs(accelerationDistance)){
            state = STATE.ACCELERATING;
        }
        //if it is within the deceleration distance of the end, then it should decelerate
        else if(abs(currentPosition) < abs(changeInPosition) - abs(deccelerationDistance)){
            state = STATE.DECELERATING;
        }
        //if it hasn't completed the loop, isn't accelerating or decelerating it should be at full power.
        else {
            state = STATE.FULL_POWER;
        }
    }

    //This creates a graph, from +deceleration to -deceleration on the x-axis with power on the y-axis
    //It then draws lines on the graph to assign power based on this positional error
    private double getVelocity(){
        //scales the graph to 1 to -1
        double positionalError;
        double scaledCurrentPosition;
        //these two add to the graph and scale to allow for a ending
        double scaleCurve;
        double addionalCurve;

        //swaps out some of the constants where accel/deccel distances changes, and accel has a starting value;
        switch (state){
            case ACCELERATING:
            positionalError = accelerationDistance - currentPosition;
            scaledCurrentPosition = positionalError / accelerationDistance;
            scaleCurve = -(maximumPower - startingPower);
            addionalCurve = startingPower;
            break;

            case DECELERATING:
            positionalError = changeInPosition - currentPosition;
            scaledCurrentPosition = positionalError / deccelerationDistance;
            scaleCurve = -(maximumPower);
            addionalCurve = 0;
            break;

            default: return 0;
        }

        //deceleration is nice, beucase it take instability and makes it stable, acceleration does the opposite,
        //so the values must be flipped around
        if(state == STATE.ACCELERATING){
            scaledCurrentPosition *= -1;
            if(scaledCurrentPosition > 1) {
                scaledCurrentPosition += 1;
            }
            else{
                scaledCurrentPosition -= 1;
            }
        }

        //has possible addions to the curve.
        switch(curve){
            case SINISUDAL: return scaleCurve * sin((PI/4) * scaledCurrentPosition) + addionalCurve;
            case PROPORTIONAL: return scaleCurve * scaledCurrentPosition + addionalCurve;
        }

        return 0;
    }

    //completes the full method, determining the power of the system at any point in its trajectory
    public double getAllVelocity(){
        switch (state){
            //at the end of the acceleration is will hit some point, the positional error is the distance to this point
            case ACCELERATING:
                return getVelocity();
                //not the positional error is the distance remaining to the target
            case DECELERATING:
                return getVelocity();
            //I feel like this makes intuitive sense
            case FULL_POWER: return maximumPower;
            //also common sense
            case COMPLETE: return 0;
        }

        return -1;
    }
}
