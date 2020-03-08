package org.firstinspires.ftc.teamcode.Control;
import static java.lang.Math.abs;

/*
The purpose of a PID or Proportional, Integral, Derivative is to correct the movement of a robot.
It takes error as an imput and attempts to correct it. This means that it is best to use it to stabilize a value
at 0. For example, you could use a PID to complete a turn, with the remaining distane as the error imput into the loop
However, this is not the best idea. It is better to use a motion profile, and compare where the robot
is to where is should be and get that value to 0.

Now, onto how the PID works, so it takes error as an imput. P is proportional to this error. Simple enough. It
moves in to complete the task. Its like if you ran at a speed proportional to how far you needed to run. Integral
calculates the total area and counteracts it. So if you are just off, the integral error will get high enough over time to
move the robot. However, if just Inegral is uses, the value oscillates, going to far one way then over correcting
and going to far the other way. To fix this, Derivative is used to compare the previous amount of error to the
current amount of error. If the amount of error changes too quickly, then derivative acts to slow it down. In practice
this stops the oscillation. Together they work together to keep the robot on track

But this only works sometimes, thats why the get value function has multipliers the values by different amounts to get
just the right balance. If the PID doesn't work its probably becuase the values are wrong.

 */
public class PID {

    //all three parts of the PID
    private double proportional, integral, derivative;
    //the parts that allow integral and derivative to work
    private double lastError, lastTime;

    //starts it up, obviously the parts of the PID are 0, and the last error and time are -1 for debugging
    public void start(){
        proportional = integral = derivative = 0;
        lastError = lastTime = -1;
    }

    //gets the pids for any combination of PID and a multiplier
    public double get(double p,double i, double d, double multiplier) {
        return  multiplier * ((p * proportional) + (i * integral) + (d * derivative));

    }

    //just in case the induviudal components are desired
    public double getProportional() { return proportional; }
    public double getIntegral() { return integral; }
    public double getDerivative() { return derivative; }

    //should the loop end? depends, So there is no error, and the error isn't changing. Shouldn't be the only condiditon.
    public boolean shouldTerminate(double maxError, double maxDifferentialError) {

        //if it hasn't started, then no
        if(lastTime == -1)return false;

        //if the error is greater than it should be, then no
        else if(abs(proportional) > maxError) return false;

        //if the error is changing more than it should
        else if(abs(derivative) < maxDifferentialError);

        //then your good
        return true;
    }

    //the meat of the class, what happens when it take on a new error
    public void onSensorChanged(double error) {

        //literally gets the current time
        double currentTime = System.currentTimeMillis();
        //finds the elapsed time in seconds.
        double elapsedTime = 0.001 * (currentTime - lastTime);

        //yup
        proportional = error;

        //integral and derivative both require another error value, so they start once it was
        if (lastError != -1) {
            //not all created equal, so that if there are more i=]puts, they are not necessarily counted more
            integral += error * elapsedTime;
            derivative = (error - lastError) / elapsedTime;
        }

        //yeah, so that the next time the loop is run, it will be good. 
        lastTime = currentTime;
        lastError = error;
    }

}