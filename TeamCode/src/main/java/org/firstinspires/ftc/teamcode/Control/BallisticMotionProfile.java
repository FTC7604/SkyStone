package org.firstinspires.ftc.teamcode.Control;


import static java.lang.Math.*;

public class BallisticMotionProfile {

    //this is the top limit of the system in regards to position, angle, etc. top and bottom limits set the bounds
    private double TOP_LIMIT;
    private double BOTTOM_LIMIT;

    //This distance value describes how close the system must be to a limit to start decelerating
    //It can be thought of as the distance needed for a full deceleration for max power
    //Larger deceldistance = flatter curve, vice versa
    private double DECELERATION_DISTANCE;

    //These parameters are about motor powers, They describe the minimum and maximum powers for the motors expressed as aboslute values
    //MIN_POWER is the minimum power needed for the system to move. When a curve is applied it will never be below MIN_POWER because otherwise the system would stall and not move at all.
    private double MIN_POWER;
    //MAX_POWER describes the maximum power that will ever be returned by a non-private method in this class. This is defaulted to 1 for full power,
    //but should be lowered if a system is under constant load so the curve can be properly shaped.
    private double MAX_POWER;

    //This is used to change the shape of the curve, See my desmos or keep at 1 for now.
    private double EXP_POWER;

    //Adjusts the power from between 0-1 to between minPower-MaxPower.
    private double POWER_ADJUSTMENT_MULTIPLIER;

    //we keep all the parameter names consistant
    public BallisticMotionProfile(final double LIMIT_1, final double LIMIT_2, final double DECELERATION_DISTANCE, final double MIN_POWER, final double EXP_POWER, final double MAX_POWER) {

        if(LIMIT_1 > LIMIT_2){
            TOP_LIMIT = LIMIT_1;
            BOTTOM_LIMIT = LIMIT_2;
        }
        else {
            TOP_LIMIT = LIMIT_2;
            BOTTOM_LIMIT = LIMIT_1;
        }

        this.DECELERATION_DISTANCE = DECELERATION_DISTANCE;
        this.MIN_POWER = MIN_POWER;
        this.EXP_POWER = EXP_POWER;
        this.MAX_POWER = MAX_POWER;
        this.POWER_ADJUSTMENT_MULTIPLIER = MAX_POWER - MIN_POWER;
    }

    //this is the heart of the ballistic class, it returns power based on the distance from something
    //PI/2 makes the period 4 times it would otherwise be, so it peaks at deceleration distance on the positive and negative ends
    //distance to limit is like the x value of the curve
    double rawCurve(double distanceToLimit){
        return sin((PI/2) * (1/DECELERATION_DISTANCE) * (distanceToLimit));
    }

    double processedCurve(double rawCurve){
         return (POWER_ADJUSTMENT_MULTIPLIER * pow(rawCurve, EXP_POWER) + MIN_POWER);
    }

    //it returns a power between minimum and maximum power that can actually be used.
    private double singleCurveLimit(double distanceToLimit) {
        //motorpower to be returned.
        double motorPower;

        //this gets set to a value between 0 and 1 based on the sin curve.
        double rawCurve = rawCurve(distanceToLimit);

        //we then adjust the power between Min and Max power and can change the shape with the exponentional power
        motorPower = processedCurve(rawCurve);

        return motorPower;
    }

    //This is very similar to the single curve version, but instead it considers more than one limit by multiplying two single curves together
    private double doubleCurveLimit(double distanceToTop, double distanceToBottom){
        //what we end up returning
        double motorPower;

        //Like rawCurve, these get set bewteen 0 and 1 as the raw curve values determined by the distance to their limits.
        //We have two this time because there are two curves
        double rawTopCurve;
        double rawBottomCurve;

        //This one is calculated when we combine the two curves, and apply power limits
        double netProcessedCurve;

        //we use these two curves to find the combined net curve based on the two distances
        //We only take the curve IF IT IS WITHIN THE DECELERATION DISTANCE
        if (distanceToTop < DECELERATION_DISTANCE) rawTopCurve = rawCurve(distanceToTop);
        else rawTopCurve = 1;
        //Otherwise we give back 1 because it isn't close enough to its limit to slow it down

        if (distanceToBottom < DECELERATION_DISTANCE) rawBottomCurve = rawCurve(distanceToBottom);
        else rawBottomCurve = 1;

        //this multiplies the two curves together, applies the floor calculations, and applies exponential multipliers if used
        netProcessedCurve = processedCurve(rawBottomCurve * rawTopCurve);

        motorPower = netProcessedCurve;

        return motorPower;//this will always be positive
    }

    //USED in TELEOP, this method keeps a system from oversteping its bounds, decelerating it as it APPROACHES one of the limits.
    //This is different than limitWithAccel because it doesn't slow the system down when it is LEAVING one of the limits.
    public double limitWithoutAccel(double currentPosition, double requestedPower) {

        //This is how we regulate the maximum motor power for a system. MAX_POWER is always a positive value less than or equal to 1. This way,
        //we will never return more than ± MAX_POWER
        requestedPower = requestedPower * MAX_POWER;

        //The distance to either the top or bottom limit, this will be set depending on which limit we are approaching
        double distanceToLimit;


        //This set of conditionals determines which way the user wants the system to move, then it decides what it needs to do to not overstep limits
        if (currentPosition < (BOTTOM_LIMIT + DECELERATION_DISTANCE)) {//if we are in the bottom threshold of our system

            distanceToLimit = currentPosition - BOTTOM_LIMIT;//the distanceToLimit is how close we are to the bottom.

            if (requestedPower < 0 && currentPosition < BOTTOM_LIMIT)//if we are trying to go down and we can't
            {
                return 0;//don't let the robot break

            } else if (requestedPower == 0) {
                return requestedPower;
            } else if (requestedPower < -singleCurveLimit(distanceToLimit)) {//all this math is really just adjusting the curve
                return -singleCurveLimit(distanceToLimit);//adjust of we have to, basically we are setting a limit
            } else {
                return requestedPower;//let free if we are going the right way and aren't in the limits
            }
        }


        ////////make sure we don't touch the top limit
        if (currentPosition > (TOP_LIMIT - DECELERATION_DISTANCE)) {
            distanceToLimit = TOP_LIMIT - currentPosition;

            if (requestedPower > 0 && currentPosition > TOP_LIMIT)//if we are trying to go up and we can't
            {
                return 0;//don't let the robot break

            } else if (requestedPower == 0) {
                return requestedPower;
            } else if (requestedPower > singleCurveLimit(distanceToLimit)) {
                return singleCurveLimit(distanceToLimit);
            } else {
                return requestedPower;
            }

        } else {//if we are just in the middle of the road then we can chill and do what we want
            return requestedPower;
        }
    }


    /////TELEOP
    ////this is a second attempt at doing and better job limiting with acceleration
    //This slows the system down depending on how close it is to the top and bottom limits, regardless of which way it is going
    //we do this using the double curve method because we are thinking about two limits
    public double V2limitWithAccel(double currentPosition, double requestedPower) {

        //This makes sure we never go above the MAX power
        requestedPower = requestedPower * MAX_POWER;

        //we first figure out how close it is to each limit
        double distanceToTop = TOP_LIMIT - currentPosition;
        double distanceToBottom = BOTTOM_LIMIT + currentPosition;

        //now we figure out the maximum allowed power for these limits, remeber that at this point it is expressed as an absolute value
        double allowedABSPower = doubleCurveLimit(distanceToTop, distanceToBottom);//the V2 version actually makes floorPower universal

        //Now that we have a maximum allowed power we need to think about what the user is asking for
        //we limit if needed, otherwise the user gets what they want
        if (abs(requestedPower) > allowedABSPower) {
            if (requestedPower > 0) return allowedABSPower;
            else if (requestedPower < 0) return -allowedABSPower;
            else return 0;
        } else {//if we arent above power limit, let it be
            return requestedPower;
        }
    }


    /////////AUTONOMOUS Call multiple times in a while loop until we reach the target position
    //we tell it where we started, where we are, and where we need to be, it is able to decide the motor power needed as a function of distance
    //This method also decides if the motor needs to be positive or negative.
    public double RunToPositionWithAccel(double startPosition, double currentPosition, double neededPosition) {//this one doesn't adhere to the top and bottom limits, will tell you a motor value when you input where it started, where it is now, and where it needs to go

        //the first step is to get the distance to the top and the distance to the bottom with regards to the bounds of this particular instance
        //they will always be positive and will be calculated slightly different depending on the instance
        double distanceToTop;
        double distanceToBottom;

        //this is the power we return for the motors for this one instance of the loop
        double curvedPower = 0;

        if (startPosition < neededPosition) {//if we need to go up, go up

            distanceToTop = (neededPosition - currentPosition);//this assumes that the top is the needed and the bottom is the start
            distanceToBottom = (currentPosition - startPosition);

            if (currentPosition > startPosition) curvedPower = doubleCurveLimit(distanceToTop, distanceToBottom);//take the curve based on where we are
            else curvedPower = MIN_POWER;
        }

        else if (startPosition > neededPosition) {//give them a negative if we need to go the other way

            distanceToTop = (startPosition - currentPosition);//this assumes that the bottom is the goal and the top is the start, meaning we need to go down
            distanceToBottom = (currentPosition - neededPosition);

            if (currentPosition < startPosition)curvedPower = -(doubleCurveLimit(distanceToTop, distanceToBottom));//we make this one negative because
            else curvedPower = -MIN_POWER;
        }

        //just enough power to keep moving such that the loop stops
        else curvedPower = 0;

        return curvedPower;
    }

    //Use this one the same as with accel, but there is no acceleration curve, meaning it starts from full speed, no matter how close the target is
    public double RunToPositionWithoutAccel(double startPosition, double currentPosition, double neededPosition) {//this one doesn't adhere to the top and bottom limits, will tell you a motor value when you input where it started, where it is now, and where it needs to go

        //the first step is to get the distance the desired position with regards to the bounds of this particular instance
        //this distance will always be positive and will be calculated slightly different depending on the instance

        //this is the distance used to calculate the allowed power for the motors. it describes how close we are to the destintion
        double distanceToLimit;

        //this is the power we return for the motors for this one instance of the loop
        double curvedPower = 0;

        if (startPosition < neededPosition) {//if we need to go up, go up

            distanceToLimit = (neededPosition - currentPosition);//this assumes that we are going from low to high encoder ticks. Otherwise the distancetotop would be flipped and the curve would be the opposite of what we want

            if (currentPosition > startPosition) curvedPower = singleCurveLimit(distanceToLimit);//take the curve based on where we are
            else curvedPower = MIN_POWER;//we have reched the destination
        }

        else if (startPosition > neededPosition) {//give them a negative if we need to go the other way

            distanceToLimit = (currentPosition - neededPosition);

            if (currentPosition < startPosition)curvedPower = -(singleCurveLimit(distanceToLimit));//we make this one negative because
            else curvedPower = -MIN_POWER;//we have reached the destination
        }

        //just enough power to keep moving such that the loop stops
        else curvedPower = 0;

        return curvedPower;
    }
}