package org.firstinspires.ftc.teamcode.Control;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class BetterBalisticProfile2 {
    private final double acceleration_distance;
    private final double deceleration_distance;

    private final double start_power;
    private final double full_power;

    public double getEnd_power() {
        return end_power;
    }

    private final double end_power;

    private final CURVE_TYPE acceleration_curve;
    private final CURVE_TYPE deceleration_curve;

    private double start_position;
    private double end_position;
    private double current_position;
    private boolean movement_is_backward;

    public BetterBalisticProfile2(
            double acceleration_distance,
            double deceleration_distance,
            double start_power,
            double full_power,
            double end_power,
            CURVE_TYPE acceleration_curve,
            CURVE_TYPE deceleration_curve) {

        this.acceleration_distance = acceleration_distance;
        this.deceleration_distance = deceleration_distance;
        this.start_power = start_power;
        this.full_power = full_power;
        this.end_power = end_power;
        this.acceleration_curve = acceleration_curve;
        this.deceleration_curve = deceleration_curve;

    }

    private double rawCurve(double x, double d, CURVE_TYPE curve_type) {

        switch (curve_type) {
            case LINEAR:
                return x / d;
            case SINUSOIDAL_NORMAL:
                return sin((PI / 2) * (x / d));
            case SINUSOIDAL_INVERTED:
                return -cos((PI / 2) * (x / d)) + 1;
            case SINUSOIDAL_NORMAL_SCURVE:
                return -.5 * cos(PI * (x / d)) + .5;
            case SINUSOIDAL_INVERTED_SCURVE:
                return (1 / PI) * Math.acos(-2 * (x / d) + 1);
        }

        return 0;
    }

    private double processedCurve(double power_adjuster, double x, double distance_zone, CURVE_TYPE curve_type, double min_power) {
        return power_adjuster * rawCurve(x, distance_zone, curve_type) + min_power;
    }

    private int invert() {
        if (movement_is_backward) return 1;
        else return -1;
    }

    private double accelerationCurve() {
        return processedCurve(
                (full_power - start_power) * invert(),
                (current_position - start_position) * invert(),
                acceleration_distance,
                acceleration_curve,
                start_power * invert()
        );
    }

    private double decelerationCurve() {
        return processedCurve(
                (full_power - end_power) * invert(),
                (end_position - current_position) * invert(),
                deceleration_distance,
                deceleration_curve,
                end_power * invert()
        );
    }

    public void setCurve(double start_position, double end_position) {
        this.start_position = start_position;
        this.end_position = end_position;
        this.movement_is_backward = start_position > end_position;
        this.current_position = start_position;
    }

    public void setCurrent_position(double current_position) {
        this.current_position = current_position;
    }

    public double getCurrent_Power() {
        boolean accelerating = abs(end_position - current_position) < deceleration_distance;
        boolean decelerating = abs(start_position - current_position) < acceleration_distance;

        if (accelerating && decelerating) {
            return accelerationCurve() * decelerationCurve() * invert();
        } else if (accelerating) {
            return accelerationCurve();
        } else if (decelerating) {
            return decelerationCurve();
        } else {
            return full_power * invert();
        }
    }

    public boolean isDone(){
        return abs(current_position - end_position) > deceleration_distance/100;
    }

    public enum CURVE_TYPE {
        LINEAR,
        SINUSOIDAL_NORMAL,
        SINUSOIDAL_INVERTED,
        SINUSOIDAL_NORMAL_SCURVE,
        SINUSOIDAL_INVERTED_SCURVE
    }
}
