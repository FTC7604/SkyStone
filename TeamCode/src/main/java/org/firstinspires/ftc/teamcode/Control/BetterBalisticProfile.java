//package org.firstinspires.ftc.teamcode.Control;
//
//import static java.lang.Math.PI;
//import static java.lang.Math.pow;
//import static java.lang.Math.sin;
//
//class BetterBalisticProfile {
//    private final double DISTANCE_OF_ACCELERATION;
//    private final double DISTANCE_OF_DECELLERATION;
//    private final double START_POWER;
//    private final double END_POWER;
//    private final double MAX_POWER;
//    private final double EXP_POWER;
//    private final double ACCELERATION_POWER_ADJUSTMENT_MULTIPLIER;
//    private final double DECELERATION_POWER_ADJUSTMENT_MULTIPLIER;
//
//    public BetterBalisticProfile(final double DISTANCE_OF_DECELLERATION,
//                                 final double DISTANCE_OF_ACCELERATION,
//                                 final double START_POWER,
//                                 final double END_POWER,
//                                 final double MAX_POWER,
//                                 final double EXP_POWER) {
//
//        this.DISTANCE_OF_ACCELERATION = DISTANCE_OF_ACCELERATION;
//        this.DISTANCE_OF_DECELLERATION = DISTANCE_OF_DECELLERATION;
//        this.START_POWER = START_POWER;
//        this.END_POWER = END_POWER;
//        this.MAX_POWER = MAX_POWER;
//        this.EXP_POWER = EXP_POWER;
//
//        ACCELERATION_POWER_ADJUSTMENT_MULTIPLIER = MAX_POWER - START_POWER;
//        DECELERATION_POWER_ADJUSTMENT_MULTIPLIER = MAX_POWER - END_POWER;
//    }
//
//    double rawCurve(double distanceToLimit, double distanceOfChange) {
//        return sin((PI / 2) * (1 / distanceOfChange) * (distanceToLimit));
//
//    }
//
//    private double processedAccelerationCurve(double distanceToLimit) {
//        return (ACCELERATION_POWER_ADJUSTMENT_MULTIPLIER * pow(distanceToLimit, DISTANCE_OF_ACCELERATION) + START_POWER);
//    }
//
//    private double processedDeccelerationCurve(double distanceFromLimit) {
//        return -(DECELERATION_POWER_ADJUSTMENT_MULTIPLIER * pow(distanceFromLimit, DISTANCE_OF_ACCELERATION) + END_POWER);
//    }
//
//    enum STATE_OF_MOVEMENT {
//        ACCELERATING,
//        FULL,
//        DECELERATING,
//        NONE
//    }
//}