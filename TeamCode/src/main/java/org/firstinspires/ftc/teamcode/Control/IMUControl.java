package org.firstinspires.ftc.teamcode.Control;

import static java.lang.Math.*;

/**
 * Created by Will McCormick, 2018-2019
 * Logic class containing math used for field-centric drive
 */
public class IMUControl {

    //contricts angle to between 0 and 2pi.
    public double positiveAngle(double angle) {
        /*All that this method does is convert this angle to something between 0 and 2PI, I don't want the robot to keep spinning around and I wouldn't be able to
        understand the meaning of the huge value. */

        //while the angle is less than 0, add 2 PI
        while (angle < 0) {
            angle += 2 * PI;
        }
        //while its greater than 2PI, subtract PI
        while (angle >= 2 * PI) {
            angle -= 2 * PI;
        }

        return angle;
    }

    //smooths data, but adding existing value to the new value at a fraction ration restrict makes angle positive
    private double[] smooth(double[] smoothData, double[] newData, double fraction, boolean restrict) {

        for (int axis = 2; axis > -1; axis--) {
            //smooths the data
            smoothData[axis] = ((1 - fraction) * smoothData[axis]) + (fraction * newData[axis]);
            //prevents the data from remaining within the bounds
            if(restrict) {
                smoothData[0] = positiveAngle(smoothData[0]);
            }
        }

        return smoothData;

    }
    private double smooth(double smoothData, double newData, double fraction, boolean restrict){
        smoothData = ((1 - fraction) * smoothData) + (fraction * newData);
        //prevents the data from remaining within the bounds
        if(restrict) {
            smoothData = positiveAngle(smoothData);
        }

        return smoothData;
    }

    //the only one that actually matters, compensates for an angle, onto some vectors
    public double[] compensate(double[] controller, double angle) {
        double x1 = controller[0];
        double y1 = -1 * controller[1];

        double x2 = 0;
        double y2 = 0;

        double[] output = new double[3];

        if(x1 != 0) {
            if(x1 > 0) {
                x2 = sqrt(x1 * x1 + y1 * y1) * cos(atan(y1 / x1) - angle);
                y2 = sqrt(x1 * x1 + y1 * y1) * sin(atan(y1 / x1) - angle);
            }
            else if(x1 < 0) {
                x2 = sqrt(x1 * x1 + y1 * y1) * cos(PI + atan(y1 / x1) - angle);
                y2 = sqrt(x1 * x1 + y1 * y1) * sin(PI + atan(y1 / x1) - angle);
            }
        }
        else{
            if(y1 > 0){
                x2 = abs(y1) * cos(PI/2 - angle);
                y2 = abs(y1) * sin(PI/2 - angle);
            }
            else if(y1 < 0){
                x2 = abs(y1) * cos(3*PI/2 - angle);
                y2 = abs(y1) * sin(3*PI/2 - angle);
            }
            else{
                x2 = 0;
                y2 = 0;
            }
        }

        output[0] = x2;
        output[1] = -1 * y2;
        output[2] = controller[2];

        return output;
    }
    public double[] velocityError(double[] desiredVelocity, double[] actualVelocity){
        /*The purpose of this method is to correct the imputs of an x-y coordinates given an x-y
        real velocity and corrects for them
         */

        //0 is the x value, and 1 is the y value

        double[] velocityError = new double[2];

        velocityError[0] = desiredVelocity[0] - actualVelocity[0];
        velocityError[1] = desiredVelocity[1] - actualVelocity[1];

        return velocityError;
    }
}