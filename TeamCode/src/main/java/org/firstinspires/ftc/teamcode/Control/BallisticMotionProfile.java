package org.firstinspires.ftc.teamcode.Control;



public class BallisticMotionProfile {

    //this is the top limit of the system in regards to position, angle, etc. top and bottom limits set the bounds
    private double TOP_LIMIT = 0;
    private double BOTTOM_LIMIT = 0;

    //This distance value describes how close the system must be to a limit to start decelerating
    //It can be thought of as the distance needed for a full deceleration for max power
    //Larger deceldistance = flatter curve, vice versa
    private double DECELERATION_DISTANCE = 0;

    //These parameters are about motor powers, They describe the minimum and maximum powers for the motors expressed as aboslute values
    //MIN_POWER is the minimum power needed for the system to move. When a curve is applied it will never be below MIN_POWER because otherwise the system would stall and not move at all.
    private double MIN_POWER = 0;
    //MAX_POWER describes the maximum power that will ever be returned by a non-private method in this class. This is defaulted to 1 for full power,
    //but should be lowered if a system is under constant load so the curve can be properly shaped.
    private double MAX_POWER = 1;

    //This is used to change the shape of the curve, See my desmos or keep at 1 for now.
    private double EXP_POWER = 1;

    //we keep all the parameter names consistant
    public BallisticMotionProfile(final double LIMIT_1, final double LIMIT_2, final double DECELERATION_DISTANCE, final double MIN_POWER, final double EXP_POWER, final double MAX_POWER) {

        //lil peice of code, so that we don't need to fiddle with the imputs
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
    }

    //This is the heart of the ballistic class. It takes in the distance to a limit,
    //and by using this value in comparison to DECELERATION_DISTANCE, it can return a positive power value between MAX_POWER, and MIN_POWER
    private double singleCurveLimit(double distanceToLimit) {
        //we eventually return this motorpower
        double motorPower;

        //this gets set to a value between 0 and 1 based on the sin curve. we get raw curve before we apply max, min, exponents, and adjustment powers
        double rawCurve;

        //this represents the highest possible power value before we consider the min power.
        final double POWER_ADJUSTMENT_MULTIPLIER = MAX_POWER - MIN_POWER;

        ///first we get the raw curve power value between 0 and 1
        rawCurve = (Math.sin(Math.PI * distanceToLimit / (2 * DECELERATION_DISTANCE)));

        //then we apply our exponent multiplier, add on the floor power, but scale it so that it doesn't exceed the MAX_POWER
        motorPower = (POWER_ADJUSTMENT_MULTIPLIER * Math.pow(rawCurve, EXP_POWER) + MIN_POWER);//use this curve to then apply additional exponents, then scale it by 0.9, then add the floor value to even it out.

        //return the curved power as a positive value, this gets made negative if needed outside of this method
        return motorPower;
    }

    //This is very similar to the single curve version, but instead it considers more than one limit.
    //The singlecurve only thinks about the distance to one limit, whereas the double curve considers the distance to both, multiplying the two curves together
    private double doubleCurveLimit(double distanceToTop, double distanceToBottom){
        //what we end up returning
        double motorPower;

        //Like rawCurve, these get set bewteen 0 and 1 as the raw curve values determined by the distance to their limits.
        //We have two this time because there are two curves
        double rawTopCurve;
        double rawBottomCurve;

        //This one is calculated when we combine the two curves, and apply power limits
        double netProcessedCurve;

        //this represents the highest possible power value before we consider the min power.
        final double POWER_ADJUSTMENT_MULTIPLIER = MAX_POWER - MIN_POWER;

        //we use these two curves to find the combined net curve based on the two distances
        //We only take the curve IF IT IS WITHIN THE DECELERATION DISTANCE
        if (distanceToTop < DECELERATION_DISTANCE) rawTopCurve = (Math.sin(Math.PI * distanceToTop / (2 * DECELERATION_DISTANCE)));
        else rawTopCurve = 1;
        //Otherwise we give back 1 because it isn't close enough to its limit to slow it down

        if (distanceToBottom < DECELERATION_DISTANCE) rawBottomCurve =   (Math.sin(Math.PI * distanceToBottom / (2 * DECELERATION_DISTANCE)));
        else rawBottomCurve = 1;

        //this multiplies the two curves together, applies the floor calculations, and applies exponential multipliers if used
        netProcessedCurve = (POWER_ADJUSTMENT_MULTIPLIER * Math.pow((rawTopCurve * rawBottomCurve), EXP_POWER)) + MIN_POWER;

        motorPower = netProcessedCurve;

        return motorPower;//this will always be positive
    }

    //USED in TELEOP, this method keeps a system from oversteping its bounds, decelerating it as it APPROACHES one of the limits.
    //This is different than limitWithAccel because it doesn't slow the system down when it is LEAVING one of the limits.
    double limitWithoutAccel(double currentPosition, double requestedPower) {

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
    double V2limitWithAccel(double currentPosition, double requestedPower) {

        //This makes sure we never go above the MAX power
        requestedPower = requestedPower * MAX_POWER;

        //we first figure out how close it is to each limit
        double distanceToTop = TOP_LIMIT - currentPosition;
        double distanceToBottom = BOTTOM_LIMIT + currentPosition;

        //now we figure out the maximum allowed power for these limits, remeber that at this point it is expressed as an absolute value
        double allowedABSPower = doubleCurveLimit(distanceToTop, distanceToBottom);//the V2 version actually makes floorPower universal

        //Now that we have a maximum allowed power we need to think about what the user is asking for
        //we limit if needed, otherwise the user gets what they want
        if (Math.abs(requestedPower) > allowedABSPower) {
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
    double RunToPositionWithoutAccel(double startPosition, double currentPosition, double neededPosition) {//this one doesn't adhere to the top and bottom limits, will tell you a motor value when you input where it started, where it is now, and where it needs to go

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