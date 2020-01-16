package org.firstinspires.ftc.teamcode.Control;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.sin;

public class BetterBalisticProfile {
    //the power that it starts/ends at when it begins the motion
    private double START_POWER;
    private double END_POWER;
    //the distance that it goes from start/end power to full speed
    private final double ACCELERATION_DISTANCE;
    private final double DECELERATION_DISTANCE;
    //the highest power that the motor will go, to prevent slippage
    private double FULL_POWER;

    private double START_POSITION;
    private double END_POSITION;
    private double CURRENT_POSITION;

    private double ACCELERATION_POWER_ADJUSTER;
    private double DECELERATION_POWER_ADJUSTER;

    public BetterBalisticProfile(
            final double ACCELERATION_DISTANCE,
            final double START_POWER,
            final double DECELERATION_DISTANCE,
            final double END_POWER,
            final double FULL_POWER) {

        this.ACCELERATION_DISTANCE = ACCELERATION_DISTANCE;
        this.DECELERATION_DISTANCE = DECELERATION_DISTANCE;
        this.START_POWER = START_POWER;
        this.END_POWER = END_POWER;
        this.FULL_POWER = FULL_POWER;

        ACCELERATION_POWER_ADJUSTER = FULL_POWER - START_POWER;
        DECELERATION_POWER_ADJUSTER = FULL_POWER - END_POWER;
    }

    //the simplest most pure curve, it will draw a line between (0,0) and (1,distanceOfChange)
    // could be a sine wave or a line
    private double rawCurve(double distanceToLimit, double distanceOfChange) {
        return sin((PI / 2) * (distanceToLimit / distanceOfChange));

    }

    //creates the curve necessary for acceleration. Modifying the function so that it starts at the start power,
    // and ends at the maximum power, and put it to the power of an an exponent
    private double processedAccelerationCurve() {
        return ((ACCELERATION_POWER_ADJUSTER) * rawCurve(CURRENT_POSITION - START_POSITION, ACCELERATION_DISTANCE) + START_POWER);
    }

    //this one works the same way as the other one
    private double processedDecelerationCurve() {
        return ((DECELERATION_POWER_ADJUSTER) * rawCurve(END_POSITION - CURRENT_POSITION, DECELERATION_DISTANCE) + END_POWER);
    }

    //sets the curve with the start and the end position of the curve. So tht it knows where to go and where to start
    //becuase it is designed to work doing forward, for negative, I trick it into going thinking that it is going forward
    //but reverse all of the other values
    public void setCurve(double startPosition, double endPosition) {
        START_POSITION = startPosition;
        END_POSITION = endPosition;

        if(startPosition < endPosition){
            START_POWER = abs(START_POWER);
            FULL_POWER = abs(FULL_POWER);
            END_POWER = abs(END_POWER);
        } else {
            START_POWER = -abs(START_POWER);
            FULL_POWER = abs(FULL_POWER);
            END_POWER = -abs(END_POWER);
        }

    }

    public double getMotorPower(double currentPosition) {

        this.CURRENT_POSITION = currentPosition;

        double accelerationValue = FULL_POWER;
        double decellerationValue = FULL_POWER;


        if (CURRENT_POSITION - START_POSITION < ACCELERATION_DISTANCE) {
            accelerationValue = processedAccelerationCurve();
        }
        if (END_POSITION - CURRENT_POSITION < DECELERATION_DISTANCE) {
            decellerationValue = processedDecelerationCurve();
        }

        return accelerationValue * decellerationValue;

    }

    public boolean isDone() {

        return (abs(CURRENT_POSITION - END_POSITION) < DECELERATION_DISTANCE/10);


    }

}