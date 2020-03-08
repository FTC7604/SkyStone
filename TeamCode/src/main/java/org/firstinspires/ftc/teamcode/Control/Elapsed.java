package org.firstinspires.ftc.teamcode.Control;

/**
 * Created by Will McCormick, 2019-2020
 * Logic class to figure out the difference between an initial value and a final value
 */
public class Elapsed {
    /*The idea behind this class is there is a bunch of stuff that needs to be kept
    track of and this class serves that purpose. It will return the elapsed time, encoders, colors
    whatever is needed.
     */

    //the starting value
    double start;

    //sets the starting value for a double
    public void start(double start){this.start = start;}

    //sets the starting value for other numerical types
    void start(int start){this.start = (double) start;}
    void start(float start){this.start = (double) start;}
    void start(long start){this.start = (double) start;}
    void start(short start){this.start = (double) start;}

    //gets the value for a double
    public double get(double current){return current - start;}

    //NOTE: the code depends on the idea that whatever type is used as an input
    //is also desired as the output, may not be the case but whatever.

    //gets the value for another numerical type
    int get(int current){return (int)((double)current - start);}
    float get(float current){return (float)((double)current - start);}
    long get(long current){return (long)((double)current - start);}
    short get(short current){return (short)((double)current - start);}
}
